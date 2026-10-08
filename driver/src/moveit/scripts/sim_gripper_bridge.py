#!/usr/bin/env python3
"""在真机夹爪MIT话题与ros2_control双指夹爪之间转换。"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_interfaces.msg import JointMitCmd, JointMotorState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimGripperBridge(Node):
    """让仿真夹爪保持与真机控制代码相同的ROS 2接口。"""

    def __init__(self) -> None:
        super().__init__("sim_gripper_bridge")
        self.open_motor_position = float(
            self.declare_parameter("open_motor_position", -4.6).value
        )
        self.closed_motor_position = float(
            self.declare_parameter("closed_motor_position", -0.1).value
        )
        self.max_finger_position = float(
            self.declare_parameter("max_finger_position", 0.045).value
        )
        self.command_duration = float(
            self.declare_parameter("command_duration", 0.10).value
        )
        self.last_motor_position: float | None = None
        self.last_feedback_time: float | None = None
        if self.open_motor_position >= self.closed_motor_position:
            raise ValueError("open_motor_position必须小于closed_motor_position")
        if self.max_finger_position <= 0.0:
            raise ValueError("max_finger_position必须大于0")

        self.command_publisher = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )
        self.feedback_publisher = self.create_publisher(
            JointMotorState,
            "/rebotarm/gripper/state",
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointMitCmd,
            "/rebotarm/gripper/cmd/mit",
            self.command_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "仿真夹爪桥接已启动：/rebotarm/gripper/cmd/mit "
            "-> /gripper_controller/joint_trajectory"
        )

    def motor_to_finger(self, motor_position: float) -> float:
        """把真机单电机角度线性换算为单侧手指直线位置。"""
        motor_position = min(
            max(motor_position, self.open_motor_position),
            self.closed_motor_position,
        )
        ratio = (
            (self.closed_motor_position - motor_position)
            / (self.closed_motor_position - self.open_motor_position)
        )
        return ratio * self.max_finger_position

    def finger_to_motor(self, finger_position: float) -> float:
        """把仿真单侧手指位置换算回真机夹爪电机角度。"""
        finger_position = min(max(finger_position, 0.0), self.max_finger_position)
        ratio = finger_position / self.max_finger_position
        return self.closed_motor_position - ratio * (
            self.closed_motor_position - self.open_motor_position
        )

    def command_callback(self, message: JointMitCmd) -> None:
        """接收真机格式MIT命令，并把其中的位置目标交给仿真夹爪。"""
        if not math.isfinite(message.pos):
            self.get_logger().warning("忽略非有限夹爪位置命令")
            return
        finger_target = self.motor_to_finger(float(message.pos))
        command = JointTrajectory()
        command.header.stamp = self.get_clock().now().to_msg()
        command.joint_names = ["gripper_joint1", "gripper_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [finger_target, finger_target]
        duration_ns = max(1, int(self.command_duration * 1_000_000_000))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        command.points = [point]
        self.command_publisher.publish(command)

    def joint_state_callback(self, message: JointState) -> None:
        """把仿真双指关节状态还原成真机格式的单电机反馈。"""
        indices = []
        for name in ("gripper_joint1", "gripper_joint2"):
            try:
                indices.append(message.name.index(name))
            except ValueError:
                return
        if any(index >= len(message.position) for index in indices):
            return

        finger_position = sum(float(message.position[index]) for index in indices) / 2.0
        motor_position = self.finger_to_motor(finger_position)

        # GenericSystem通常只复制位置命令，velocity状态长期为0，不能用于
        # 上层堵转判断。这里根据相邻两帧仿真位置计算等效电机角速度。
        feedback_time = time.monotonic()
        motor_velocity = 0.0
        if self.last_motor_position is not None and self.last_feedback_time is not None:
            elapsed = feedback_time - self.last_feedback_time
            if elapsed > 1.0e-6:
                motor_velocity = (
                    motor_position - self.last_motor_position
                ) / elapsed
        self.last_motor_position = motor_position
        self.last_feedback_time = feedback_time

        feedback = JointMotorState()
        feedback.header = message.header
        feedback.joint_name = "gripper"
        feedback.position = motor_position
        feedback.velocity = motor_velocity
        feedback.torque = 0.0
        feedback.status_code = 0
        self.feedback_publisher.publish(feedback)


def main() -> None:
    """运行仿真夹爪接口桥接节点。"""
    rclpy.init()
    node = SimGripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
