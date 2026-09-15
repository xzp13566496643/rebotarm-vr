from __future__ import annotations

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardwaremanager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .actions import ArmActions
from .publishers import JointStatePublisher
from .services import ArmServices


class reBotArmController(Node):
    def __init__(self) -> None:
        """创建真机 ROS2 驱动节点，连接硬件并注册反馈、服务、动作和低层命令接口。"""
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter(
            "sdk_root",
            "/home/xuzhanpeng/ws/arm_controll/driver/reBotArm_control_py",
        )
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("disable_after_safe_home", True)
        self.declare_parameter("servo_velocity_limit", 1.0)
        self.declare_parameter("servo_max_step", 0.0)
        self.declare_parameter("servo_velocity_lookahead", 0.15)
        self.declare_parameter("servo_max_lookahead_step", 0.05)

        hardware_config = self.get_parameter("hardware_config").value or None
        sdk_root = str(self.get_parameter("sdk_root").value or "")
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        servo_velocity_limit = float(self.get_parameter("servo_velocity_limit").value)
        servo_max_step = float(self.get_parameter("servo_max_step").value)
        servo_velocity_lookahead = float(
            self.get_parameter("servo_velocity_lookahead").value
        )
        servo_max_lookahead_step = float(
            self.get_parameter("servo_max_lookahead_step").value
        )
        self.disable_after_safe_home = bool(
            self.get_parameter("disable_after_safe_home").value
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            sdk_root=sdk_root,
            model=model,
            channel=channel,
        )
        self.hardware.connect()

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
            servo_velocity_limit,
            servo_max_step,
            servo_velocity_lookahead,
            servo_max_lookahead_step,
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        """立即发布一次驱动状态；可选择是否同时读取各电机状态码。"""
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def shutdown(self) -> None:
        """执行驱动关闭流程：安全回位，并按参数决定是否失能电机。"""
        self.hardware.shutdown(
            disable_after_safe_home=self.disable_after_safe_home,
        )


def main(args=None) -> None:
    """运行多线程 ROS2 驱动；退出时保证调用硬件关闭流程。"""
    rclpy.init(args=args)
    node = reBotArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
