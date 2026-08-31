#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rebotarm_msgs.msg import JointPosVelCmd
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def q_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q) if n > 1e-12 else (0.0, 0.0, 0.0, 1.0)


def q_inverse(q):
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def clamp_norm(v, limit):
    n = math.sqrt(sum(x * x for x in v))
    if limit > 0.0 and n > limit:
        return tuple(x * limit / n for x in v)
    return tuple(v)


def map_vr_vector(v):
    # OpenXR (+X right, +Y up, -Z forward) -> reBot base (+X forward, +Y left, +Z up)
    return (-v[2], -v[0], v[1])


class VrPoseTargetServo(Node):
    def __init__(self):
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
        self.gripper_motor_vlim = self.declare_parameter("gripper_motor_vlim", 2.0).value

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.hardware_gripper_pub = self.create_publisher(
            JointPosVelCmd, "/rebotarm/gripper/cmd/pos_vel", 10)
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
            "VR cumulative pose target ready: hold left trigger; right pose sets a persistent TCP target")

    def pose_cb(self, msg):
        self.hand_pose = msg.pose
        self.last_hand_time = self.get_clock().now()
        if self.engaged and self.hand_start is not None and self.robot_start is not None:
            self.update_target()

    def clutch_cb(self, msg):
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
        trigger = max(0.0, min(float(msg.data), 1.0))
        if self.hardware_gripper:
            if trigger >= 0.70:
                target = self.gripper_motor_closed
            elif trigger <= 0.30:
                target = self.gripper_motor_open
            else:
                return
            if self.last_gripper is not None and abs(target - self.last_gripper) < 0.02:
                return
            cmd = JointPosVelCmd()
            cmd.pos = target
            cmd.vlim = self.gripper_motor_vlim
            self.hardware_gripper_pub.publish(cmd)
            self.last_gripper = target
            return

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

    def lookup_tcp(self):
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "gripper_tcp", Time())
        except TransformException as exc:
            self.get_logger().warning(f"TCP transform unavailable: {exc}")
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def update_target(self):
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
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.twist_pub.publish(cmd)


def main():
    rclpy.init()
    node = VrPoseTargetServo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
