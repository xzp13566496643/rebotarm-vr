#!/usr/bin/env python3
"""Plan, and optionally execute, a 10 cm upward TCP motion on the real arm.

Required running nodes:
  1. rebotarm_bringup/driver.launch.py
  2. rebotarm_moveit_config/hardware.launch.py

The script is plan-only by default. Pass --execute to send the planned
trajectory to MoveIt's /execute_trajectory action.
"""

from __future__ import annotations

import argparse
import sys
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
BASE_FRAME = "base_link"
TCP_FRAME = "gripper_tcp"
GROUP_NAME = "arm"


class MoveTcpUp:
    def __init__(self, dz: float, execute: bool) -> None:
        self.node = rclpy.create_node("move_tcp_up")
        self.dz = float(dz)
        self.should_execute = bool(execute)
        self.latest_positions: dict[str, float] = {}

        self.node.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.plan_client = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.execute_client = ActionClient(
            self.node, ExecuteTrajectory, "/execute_trajectory"
        )

    def _joint_state_cb(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            self.latest_positions[name] = float(position)

    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        return future.done()

    def _wait_current_joints(self, timeout: float = 5.0) -> list[float] | None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.latest_positions for name in ARM_JOINTS):
                return [self.latest_positions[name] for name in ARM_JOINTS]
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().error("没有收到完整的 /rebotarm/joint_states")
        return None

    def _current_tcp_target(self, timeout: float = 5.0) -> PoseStamped | None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TCP_FRAME, Time()
                )
            except TransformException:
                continue

            pose = PoseStamped()
            pose.header.frame_id = BASE_FRAME
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z + self.dz
            # The goal uses only this position. TCP orientation is intentionally
            # left unconstrained so MoveIt can choose any reachable wrist pose.
            pose.pose.orientation.w = 1.0
            return pose

        self.node.get_logger().error(
            f"无法获取 TF: {BASE_FRAME} -> {TCP_FRAME}"
        )
        return None

    @staticmethod
    def _robot_state(values: list[float]) -> RobotState:
        return RobotState(
            is_diff=False,
            joint_state=JointState(name=ARM_JOINTS, position=values),
        )

    @staticmethod
    def _position_goal(target: PoseStamped) -> Constraints:
        tolerance_sphere = SolidPrimitive()
        tolerance_sphere.type = SolidPrimitive.SPHERE
        tolerance_sphere.dimensions = [0.01]

        center = target.pose
        center.orientation.x = 0.0
        center.orientation.y = 0.0
        center.orientation.z = 0.0
        center.orientation.w = 1.0

        return Constraints(
            position_constraints=[
                PositionConstraint(
                    header=target.header,
                    link_name=TCP_FRAME,
                    constraint_region=BoundingVolume(
                        primitives=[tolerance_sphere],
                        primitive_poses=[center],
                    ),
                    weight=1.0,
                )
            ]
        )

    def _plan(self, start: list[float], target: PoseStamped):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = GROUP_NAME
        motion.pipeline_id = "ompl"
        motion.planner_id = "RRTConnectkConfigDefault"
        motion.allowed_planning_time = 10.0
        motion.num_planning_attempts = 20
        motion.max_velocity_scaling_factor = 0.05
        motion.max_acceleration_scaling_factor = 0.05
        motion.start_state = self._robot_state(start)
        motion.goal_constraints = [self._position_goal(target)]

        future = self.plan_client.call_async(request)
        if not self._wait_future(future, 25.0):
            self.node.get_logger().error("MoveIt 规划请求超时")
            return None

        response = future.result()
        result = response.motion_plan_response if response is not None else None
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = result.error_code.val if result is not None else "empty"
            self.node.get_logger().error(f"MoveIt 规划失败，错误码: {code}")
            return None
        return result.trajectory

    def _execute(self, trajectory) -> bool:
        send_future = self.execute_client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)
        )
        if not self._wait_future(send_future, 5.0):
            self.node.get_logger().error("发送执行目标超时")
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("MoveIt 拒绝执行轨迹")
            return False

        result_future = handle.get_result_async()
        if not self._wait_future(result_future, 30.0):
            self.node.get_logger().error("等待轨迹执行结果超时")
            return False

        wrapped = result_future.result()
        result = wrapped.result if wrapped is not None else None
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = result.error_code.val if result is not None else "empty"
            self.node.get_logger().error(f"轨迹执行失败，错误码: {code}")
            return False
        return True

    def run(self) -> bool:
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("/plan_kinematic_path 不可用")
            return False
        if self.should_execute and not self.execute_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.node.get_logger().error("/execute_trajectory 不可用")
            return False

        current = self._wait_current_joints()
        target_pose = self._current_tcp_target()
        if current is None or target_pose is None:
            return False

        p = target_pose.pose.position
        self.node.get_logger().info(
            f"目标 TCP ({BASE_FRAME}): x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}"
        )
        self.node.get_logger().info("末端姿态不约束，位置容差半径为 0.01 m")

        trajectory = self._plan(current, target_pose)
        if trajectory is None:
            return False
        point_count = len(trajectory.joint_trajectory.points)
        self.node.get_logger().info(f"规划成功，轨迹点数量: {point_count}")

        if not self.should_execute:
            self.node.get_logger().info(
                "当前为只规划模式，机械臂不会运动；确认后添加 --execute"
            )
            return True

        self.node.get_logger().warn(
            f"开始执行：TCP 目标高度增加 {self.dz:.3f} m"
        )
        if not self._execute(trajectory):
            return False
        self.node.get_logger().info("轨迹执行成功")
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dz",
        type=float,
        default=0.10,
        help="TCP 在 base_link Z 方向的增量，单位 m（默认 0.10）",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际执行规划轨迹；不加此参数时只规划",
    )
    return parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])


def main() -> int:
    rclpy.init()
    args = parse_args()
    app = MoveTcpUp(args.dz, args.execute)
    try:
        return 0 if app.run() else 1
    except KeyboardInterrupt:
        app.node.get_logger().warn("用户终止")
        return 130
    finally:
        app.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
