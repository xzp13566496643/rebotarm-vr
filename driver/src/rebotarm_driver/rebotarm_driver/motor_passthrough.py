from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rebotarm_interfaces.msg import (
    JointMitCmd,
    JointPosVelCmd,
)
from trajectory_msgs.msg import JointTrajectory


class MotorPassthrough:
    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        arbitration: str,
        servo_velocity_limit: float,
        servo_max_step: float,
        servo_velocity_lookahead: float,
        servo_max_lookahead_step: float,
    ) -> None:
        """建立 ROS2 低层电机命令订阅，并把消息转交给 HardwareManager。"""
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []
        self._servo_velocity_limit = servo_velocity_limit
        self._servo_max_step = servo_max_step
        self._servo_velocity_lookahead = servo_velocity_lookahead
        self._servo_max_lookahead_step = servo_max_lookahead_step

        joint_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, name, msg: hw.send_joint_mit_cmd(
                    name,
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, name, msg: hw.send_joint_pos_vel_cmd(
                    name,
                    msg.pos,
                    msg.vlim,
                ),
            ),
        )
        gripper_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, msg: hw.send_gripper_mit_cmd(
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                # 对应 /rebotarm/gripper/cmd/pos_vel：携带目标角度 pos 和速度上限 vlim。
                lambda hw, msg: hw.send_gripper_pos_vel_cmd(msg.pos, msg.vlim),
            ),
        )

        for joint_name in hardware.joint_names:
            for msg_type, label, command in joint_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/joints/{joint_name}/{label}",
                    self._make_joint_callback(
                        joint_name,
                        label,
                        command,
                    ),
                    qos,
                )
        if hardware.has_gripper:
            for msg_type, label, command in gripper_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/gripper/{label}",
                    self._make_gripper_callback(label, command),
                    qos,
                )

        self._subscribe(
            JointTrajectory,
            f"/{namespace}/servo_joint_trajectory",
            self._servo_trajectory_callback,
            qos,
        )

    def _subscribe(self, msg_type, topic: str, callback, qos: QoSProfile) -> None:
        """创建一个可靠 QoS 的 ROS2 订阅并保存对象，防止订阅被垃圾回收。"""
        self._subscriptions.append(
            self._node.create_subscription(
                msg_type,
                topic,
                callback,
                qos,
                callback_group=self._node.reentrant_group,
            )
        )

    def _make_joint_callback(self, joint_name: str, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/joints/{joint_name}/{label}",
                allow_preempt=True,
            ):
                return

            try:
                command(self._hardware, joint_name, msg)
            except Exception as exc:
                self._node.get_logger().warn(
                    f"joint {label} failed for {joint_name}: {exc}"
                )
            finally:
                self._node.publish_arm_status()

        return _callback

    def _make_gripper_callback(self, label: str, command) -> object:
        """生成夹爪话题回调：先做控制仲裁，再调用对应的硬件命令函数。"""
        def _callback(msg) -> None:
            # 安全回零、重力补偿等状态下拒绝互相冲突的低层夹爪命令。
            if not self._can_send_lowlevel(
                f"/gripper/{label}",
                allow_preempt=False,
            ):
                return

            try:
                # POS_VEL 时最终调用 HardwareManager.send_gripper_pos_vel_cmd()。
                command(self._hardware, msg)
            except Exception as exc:
                self._node.get_logger().warn(f"gripper {label} failed: {exc}")
            finally:
                self._node.publish_arm_status()

        return _callback

    def _servo_trajectory_callback(self, msg: JointTrajectory) -> None:
        """接收 MoveIt Servo 六关节轨迹点并转交真机 POS_VEL 接口。"""
        if not self._can_send_lowlevel("/servo_joint_trajectory", allow_preempt=True):
            return
        if not msg.points:
            self._node.get_logger().warn("rejecting empty Servo trajectory")
            return
        point = msg.points[-1]
        velocities = list(point.velocities)
        if velocities and len(velocities) != len(msg.joint_names):
            self._node.get_logger().warn(
                "rejecting Servo trajectory with mismatched joint velocities"
            )
            return
        try:
            self._hardware.send_joint_group_pos_vel_cmd(
                list(msg.joint_names),
                list(point.positions),
                velocities,
                self._servo_velocity_limit,
                self._servo_max_step,
                self._servo_velocity_lookahead,
                self._servo_max_lookahead_step,
            )
        except Exception as exc:
            self._node.get_logger().warn(f"Servo trajectory rejected: {exc}")
        finally:
            self._node.publish_arm_status()

    def _can_send_lowlevel(self, label: str, *, allow_preempt: bool) -> bool:
        """根据驱动状态机仲裁，决定当前低层命令允许、拒绝还是抢占。"""
        state = self._hardware.state_machine
        if state in ("GRAVITY_COMP", "SAFE_HOMING"):
            self._node.get_logger().warn(f"rejecting {label} in state {state}")
            return False
        if state == "TRAJ_RUNNING":
            if self._arbitration == "reject" or not allow_preempt:
                self._node.get_logger().warn(
                    f"rejecting {label} while trajectory is running"
                )
                return False
            self._node.get_logger().warn(
                f"preempting trajectory for {label}"
            )
            self._hardware.stop_motion()
        return True
