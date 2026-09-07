#!/usr/bin/env python3
import math

import rclpy
from gripper_grasp_controller import GripperGraspController, GripperMitCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rebotarm_msgs.msg import JointMitCmd, JointMotorState, JointPosVelCmd
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def q_normalize(q):
    """归一化四元数；输入接近零时返回单位四元数。"""
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q) if n > 1e-12 else (0.0, 0.0, 0.0, 1.0)


def q_inverse(q):
    """计算单位四元数的逆，用于求手柄相对旋转。"""
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply(a, b):
    """计算两个四元数的乘积并归一化结果。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def clamp_norm(v, limit):
    """保持向量方向不变，将模长限制到指定上限。"""
    n = math.sqrt(sum(x * x for x in v))
    if limit > 0.0 and n > limit:
        return tuple(x * limit / n for x in v)
    return tuple(v)


def map_vr_vector(v):
    """把 OpenXR 手柄坐标轴映射到机械臂 base_link 坐标轴。"""
    # OpenXR (+X right, +Y up, -Z forward) -> reBot base (+X forward, +Y left, +Z up)
    return (-v[2], -v[0], v[1])


class VrPoseTargetServo(Node):
    def __init__(self):
        """创建 VR 订阅、Servo/夹爪发布器以及 50 Hz 控制定时器。"""
        super().__init__("vr_pose_target_servo")
        self.position_scale = self.declare_parameter("position_scale", 0.8).value
        self.orientation_scale = self.declare_parameter("orientation_scale", 1.0).value
        self.position_gain = self.declare_parameter("position_gain", 3.0).value
        self.orientation_gain = self.declare_parameter("orientation_gain", 3.0).value
        self.max_linear_speed = self.declare_parameter("max_linear_speed", 0.4).value
        self.max_angular_speed = self.declare_parameter("max_angular_speed", 0.8).value
        self.trigger_threshold = self.declare_parameter("trigger_threshold", 0.5).value
        self.input_timeout = self.declare_parameter("input_timeout", 0.2).value
        self.hardware_gripper = self.declare_parameter("hardware_gripper", False).value
        self.gripper_motor_open = self.declare_parameter("gripper_motor_open", -5.0).value
        self.gripper_motor_closed = self.declare_parameter("gripper_motor_closed", 0.0).value
        self.gripper_control_mode = str(
            self.declare_parameter("gripper_control_mode", "pos_vel").value
        ).strip().lower()
        self.gripper_motor_vlim = self.declare_parameter("gripper_motor_vlim", 2.0).value
        self.gripper_mit_kp = self.declare_parameter("gripper_mit_kp", 5.0).value
        self.gripper_mit_kd = self.declare_parameter("gripper_mit_kd", 1.0).value
        self.gripper_opening_torque_limit = self.declare_parameter(
            "gripper_opening_torque_limit", 1.0).value
        self.gripper_close_kd = self.declare_parameter("gripper_close_kd", 0.5).value
        self.gripper_close_torque = self.declare_parameter("gripper_close_torque", 0.5).value
        self.gripper_hold_torque = self.declare_parameter("gripper_hold_torque", 0.30).value
        self.gripper_tau_max = self.declare_parameter("gripper_tau_max", 1.5).value
        self.gripper_stall_velocity = self.declare_parameter("gripper_stall_velocity", 0.05).value
        self.gripper_stall_samples = self.declare_parameter("gripper_stall_samples", 1).value
        if self.gripper_control_mode not in ("mit", "pos_vel", "grasp_mit"):
            raise ValueError(
                "gripper_control_mode must be 'mit', 'pos_vel' or 'grasp_mit', got "
                f"{self.gripper_control_mode!r}"
            )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.hardware_gripper_pos_vel_pub = self.create_publisher(
            JointPosVelCmd, "/rebotarm/gripper/cmd/pos_vel", 10)
        self.hardware_gripper_mit_pub = self.create_publisher(
            JointMitCmd, "/rebotarm/gripper/cmd/mit", 10)
        self.gripper_feedback_initialized = not self.hardware_gripper
        self.gripper_initial_position = None
        self.gripper_trigger_closed = None
        self.gripper_grasp_controller = None
        self.gripper_grasp_last_state = None
        if self.hardware_gripper and self.gripper_control_mode == "grasp_mit":
            self.gripper_grasp_controller = GripperGraspController(
                open_position=self.gripper_motor_open,
                closed_position=self.gripper_motor_closed,
                move_kp=self.gripper_mit_kp,
                move_kd=self.gripper_mit_kd,
                opening_torque_limit=self.gripper_opening_torque_limit,
                close_kd=self.gripper_close_kd,
                close_torque=self.gripper_close_torque,
                hold_torque=self.gripper_hold_torque,
                tau_max=self.gripper_tau_max,
                stall_velocity=self.gripper_stall_velocity,
                stall_samples=self.gripper_stall_samples,
            )
        if self.hardware_gripper:
            self.create_subscription(
                JointMotorState,
                "/rebotarm/gripper/state",
                self.gripper_state_cb,
                qos_profile_sensor_data,
            )
        self.create_subscription(PoseStamped, "/pico_right_controller/pose", self.pose_cb, 10)
        self.create_subscription(Float32, "/pico_left_controller/trigger", self.clutch_cb, 10)
        self.create_subscription(Float32, "/pico_right_controller/trigger", self.gripper_cb, 10)

        self.hand_pose = None
        self.last_hand_time = None
        self.engaged = False
        self.hand_start = None
        self.robot_start = None
        self.target = None
        self.last_gripper = None
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            "VR cumulative pose target ready: hold left trigger; right pose sets a "
            f"persistent TCP target; hardware gripper mode={self.gripper_control_mode}")

    def gripper_state_cb(self, msg):
        """初始化夹爪；grasp_mit模式还持续缓存反馈用于检测物体接触。"""
        if self.gripper_grasp_controller is not None:
            first = self.gripper_grasp_controller.update_feedback(
                msg.position, msg.velocity, msg.torque
            )
            if first:
                self.gripper_initial_position = float(msg.position)
                self.publish_gripper_mit_command(
                    self.gripper_grasp_controller.hold_current_command()
                )
                self.gripper_feedback_initialized = True
                self.get_logger().info(
                    f"Grasp MIT initialized from feedback at {msg.position:.4f} rad"
                )
            return

        if self.gripper_feedback_initialized:
            return
        position = float(msg.position)
        if not math.isfinite(position):
            self.get_logger().warning("Ignoring non-finite initial gripper feedback")
            return

        # 首条低层命令保持反馈到的真实位置，避免切换 MIT/POS_VEL 时使用盲目绝对目标。
        self.gripper_initial_position = position
        self.publish_hardware_gripper(position)
        self.gripper_feedback_initialized = True
        self.get_logger().info(
            f"Gripper initialized from feedback at {position:.4f} rad; "
            "subsequent commands no longer depend on feedback"
        )

    def pose_cb(self, msg):
        """缓存右手柄绝对位姿，离合接合时同步更新累计目标。"""
        self.hand_pose = msg.pose
        self.last_hand_time = self.get_clock().now()
        if self.engaged and self.hand_start is not None and self.robot_start is not None:
            self.update_target()

    def clutch_cb(self, msg):
        """由左扳机接合控制，并记录手柄与真机TCP的起始基准。"""
        requested = msg.data >= self.trigger_threshold
        if requested and not self.engaged:
            if self.hand_pose is None:
                self.get_logger().warning("Cannot engage: no right-controller pose")
                return
            robot = self.lookup_tcp()
            if robot is None:
                return
            hp = self.hand_pose.position
            hq = self.hand_pose.orientation
            self.hand_start = ((hp.x, hp.y, hp.z), (hq.x, hq.y, hq.z, hq.w))
            self.robot_start = robot
            self.target = robot
            self.engaged = True
            self.get_logger().info("VR cumulative clutch engaged")
        elif not requested and self.engaged:
            self.engaged = False
            self.hand_start = None
            self.robot_start = None
            self.target = None
            self.publish_zero()
            self.get_logger().info("VR cumulative clutch released")

    def gripper_cb(self, msg):
        """把右手扳机值转换为夹爪开/合命令，并发布给仿真或真机驱动。"""
        # PICO 发布 Float32；先限制到扳机的有效范围 [0, 1]。
        trigger = max(0.0, min(float(msg.data), 1.0))
        if self.hardware_gripper:
            if not self.gripper_feedback_initialized:
                # 启动阶段禁止在不知道真实位置时发送开/合绝对目标。
                return
            if self.gripper_grasp_controller is not None:
                if trigger >= 0.70 and self.gripper_trigger_closed is not True:
                    self.gripper_trigger_closed = True
                    if self.gripper_grasp_controller.request_close():
                        self.get_logger().info("Gripper grasp: CLOSING")
                elif trigger <= 0.30 and self.gripper_trigger_closed is not False:
                    self.gripper_trigger_closed = False
                    if self.gripper_grasp_controller.request_open():
                        self.get_logger().info("Gripper grasp: OPENING")
                return
            # 真机采用带回差的二值控制，避免扳机在阈值附近抖动时反复开合。
            if trigger >= 0.70:
                target = self.gripper_motor_closed
            elif trigger <= 0.30:
                target = self.gripper_motor_open
            else:
                return
            if self.last_gripper is not None and abs(target - self.last_gripper) < 0.02:
                return
            target = max(
                min(target, self.gripper_motor_closed), self.gripper_motor_open)
            self.publish_hardware_gripper(target)
            self.last_gripper = target
            return

        # 仿真夹爪不是单电机接口，而是向两个手指关节发送 JointTrajectory。
        target = 0.045 * (1.0 - trigger)
        if self.last_gripper is not None and abs(target - self.last_gripper) < 0.001:
            return
        cmd = JointTrajectory()
        cmd.joint_names = ["gripper_joint1", "gripper_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        point.time_from_start.nanosec = 100000000
        cmd.points = [point]
        self.gripper_pub.publish(cmd)
        self.last_gripper = target

    def publish_hardware_gripper(self, target: float) -> None:
        """根据启动参数选择 MIT 或 POS_VEL，向真机夹爪发布一种低层命令。"""
        if self.gripper_control_mode == "mit":
            # MIT由电机内部编码器闭环计算力矩；这里不使用上位机反馈插值。
            cmd = JointMitCmd()
            cmd.pos = float(target)
            cmd.vel = 0.0
            cmd.kp = max(0.0, float(self.gripper_mit_kp))
            cmd.kd = max(0.0, float(self.gripper_mit_kd))
            cmd.tau = 0.0
            cmd.stamp = self.get_clock().now().to_msg()
            self.hardware_gripper_mit_pub.publish(cmd)
            return

        # POS_VEL由电机内部位置环/速度环运动到目标，并遵守vlim速度上限。
        cmd = JointPosVelCmd()
        cmd.pos = float(target)
        cmd.vlim = max(0.0, float(self.gripper_motor_vlim))
        self.hardware_gripper_pos_vel_pub.publish(cmd)

    def publish_gripper_mit_command(self, command: GripperMitCommand) -> None:
        """把抓取状态机生成的MIT五元组转换成ROS消息并发布。"""
        cmd = JointMitCmd()
        cmd.pos = float(command.pos)
        cmd.vel = float(command.vel)
        cmd.kp = float(command.kp)
        cmd.kd = float(command.kd)
        cmd.tau = float(command.tau)
        cmd.stamp = self.get_clock().now().to_msg()
        self.hardware_gripper_mit_pub.publish(cmd)

    def update_grasp_gripper(self) -> None:
        """以50 Hz推进夹爪抓取状态机并输出命令，同时记录状态变化。"""
        if self.gripper_grasp_controller is None:
            return
        command = self.gripper_grasp_controller.tick()
        state = self.gripper_grasp_controller.state
        if state != self.gripper_grasp_last_state:
            self.gripper_grasp_last_state = state
            if state == GripperGraspController.HOLDING:
                self.get_logger().info(
                    "Gripper grasp: contact detected, HOLDING at "
                    f"{self.gripper_grasp_controller.contact_position:.4f} rad"
                )
            elif state == GripperGraspController.EMPTY:
                self.get_logger().info("Gripper grasp: EMPTY (closed without object)")
            elif state == GripperGraspController.FEEDBACK_TIMEOUT:
                self.get_logger().error(
                    "Gripper grasp: feedback timeout, closing torque removed"
                )
        if command is not None:
            self.publish_gripper_mit_command(command)

    def lookup_tcp(self):
        """从 TF 查询 base_link 到 gripper_tcp 的当前末端位姿。"""
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "gripper_tcp", Time())
        except TransformException as exc:
            self.get_logger().warning(f"TCP transform unavailable: {exc}")
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def update_target(self):
        """把手柄相对基准的累计变化映射为持久的机械臂目标位姿。"""
        hp, hq_msg = self.hand_pose.position, self.hand_pose.orientation
        h0p, h0q = self.hand_start
        dp = map_vr_vector((hp.x - h0p[0], hp.y - h0p[1], hp.z - h0p[2]))
        p0, q0 = self.robot_start

        hq = (hq_msg.x, hq_msg.y, hq_msg.z, hq_msg.w)
        q_delta_vr = q_multiply(hq, q_inverse(h0q))
        # A basis change maps the quaternion vector part like an angular vector.
        mapped_xyz = map_vr_vector(q_delta_vr[:3])
        q_delta_robot = q_normalize((
            mapped_xyz[0] * self.orientation_scale,
            mapped_xyz[1] * self.orientation_scale,
            mapped_xyz[2] * self.orientation_scale,
            q_delta_vr[3],
        ))
        self.target = (
            (p0[0] + self.position_scale * dp[0],
             p0[1] + self.position_scale * dp[1],
             p0[2] + self.position_scale * dp[2]),
            q_multiply(q_delta_robot, q0),
        )

    def control_loop(self):
        """以 50 Hz 计算目标误差，生成受限 Twist 并交给 MoveIt Servo。"""
        self.update_grasp_gripper()
        if not self.engaged or self.target is None or self.last_hand_time is None:
            self.publish_zero()
            return
        if (self.get_clock().now() - self.last_hand_time).nanoseconds * 1e-9 > self.input_timeout:
            self.publish_zero()
            return
        current = self.lookup_tcp()
        if current is None:
            self.publish_zero()
            return
        p, q = current
        pt, qt = self.target
        linear = clamp_norm(tuple(self.position_gain * (pt[i] - p[i]) for i in range(3)),
                            self.max_linear_speed)

        q_error = q_multiply(qt, q_inverse(q))
        if q_error[3] < 0.0:
            q_error = tuple(-x for x in q_error)
        vector_norm = math.sqrt(sum(x * x for x in q_error[:3]))
        angular = (0.0, 0.0, 0.0)
        if vector_norm > 1e-9:
            angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(q_error[3], 1.0)))
            angular = tuple(self.orientation_gain * angle * x / vector_norm for x in q_error[:3])
            angular = clamp_norm(angular, self.max_angular_speed)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = linear
        cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z = angular
        self.twist_pub.publish(cmd)

    def publish_zero(self):
        """发布零 Twist，要求 MoveIt Servo 停止继续移动。"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.twist_pub.publish(cmd)


def main():
    """初始化并运行 VR 累计位姿控制节点。"""
    rclpy.init()
    node = VrPoseTargetServo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
