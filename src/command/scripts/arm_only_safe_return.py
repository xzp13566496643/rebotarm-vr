#!/usr/bin/env python3
"""Slowly return only the six arm joints through the low-level Servo stream."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TARGET = [0.0, -0.75, -0.55, 0.0, 0.0, 0.0]
DURATION = 12.0
RATE = 50.0


class ArmOnlySafeReturn(Node):
    def __init__(self) -> None:
        super().__init__("arm_only_safe_return")
        self.state = None
        self.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self.state_cb,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            JointTrajectory, "/rebotarm/servo_joint_trajectory", 10)

    def state_cb(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINTS):
            self.state = [float(values[name]) for name in JOINTS]

    def publish_target(self, positions, velocities) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = list(velocities)
        point.time_from_start.nanosec = 20_000_000
        msg.points = [point]
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ArmOnlySafeReturn()
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and node.state is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.state is None:
            node.get_logger().error(
                "No /rebotarm/joint_states feedback in 5 seconds; no command sent")
            return

        start = node.state[:]
        node.get_logger().info(
            f"Returning arm only over {DURATION:.1f}s: start={start}, target={TARGET}")
        start_time = time.monotonic()
        period = 1.0 / RATE

        while rclpy.ok():
            ratio = min((time.monotonic() - start_time) / DURATION, 1.0)
            blend = ratio * ratio * (3.0 - 2.0 * ratio)
            blend_rate = 6.0 * ratio * (1.0 - ratio) / DURATION
            positions = [a + (b - a) * blend for a, b in zip(start, TARGET)]
            velocities = [(b - a) * blend_rate for a, b in zip(start, TARGET)]
            node.publish_target(positions, velocities)
            rclpy.spin_once(node, timeout_sec=0.0)
            if ratio >= 1.0:
                break
            time.sleep(period)

        for _ in range(20):
            node.publish_target(TARGET, [0.0] * 6)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        node.get_logger().info("Safe-return commands complete; verify the real arm stopped")
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
