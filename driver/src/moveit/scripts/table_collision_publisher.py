#!/usr/bin/env python3
"""向MoveIt规划场景发布带底座开口的桌面碰撞体。"""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


class TableCollisionPublisher(Node):
    """用四个长方体组成桌面，并将桌面上表面放置在指定Z高度。"""

    def __init__(self) -> None:
        super().__init__("table_collision_publisher")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("table_surface_z", 0.0)
        self.declare_parameter("table_size", 4.0)
        self.declare_parameter("table_base_cutout_size", 0.40)
        self.declare_parameter("table_thickness", 0.04)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.surface_z = float(self.get_parameter("table_surface_z").value)
        self.table_size = float(self.get_parameter("table_size").value)
        self.cutout_size = float(
            self.get_parameter("table_base_cutout_size").value
        )
        self.thickness = float(self.get_parameter("table_thickness").value)

        if self.table_size <= 0.0:
            raise ValueError("table_size必须大于0")
        if self.thickness <= 0.0:
            raise ValueError("table_thickness必须大于0")
        if not 0.0 < self.cutout_size < self.table_size:
            raise ValueError(
                "table_base_cutout_size必须大于0并且小于table_size"
            )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            CollisionObject, "/collision_object", qos
        )

        # 延迟一小段时间发布，让PlanningSceneMonitor先完成初始化；
        # TRANSIENT_LOCAL还会为稍后加入的订阅者保留最后一条桌面消息。
        self.publish_timer = self.create_timer(0.5, self.publish_table_once)

    @staticmethod
    def make_box(size_x: float, size_y: float, size_z: float) -> SolidPrimitive:
        """创建一个指定长宽高的MoveIt长方体碰撞几何。"""
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]
        return primitive

    @staticmethod
    def make_pose(x: float, y: float, z: float) -> Pose:
        """创建无旋转的碰撞体位姿。"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def publish_table_once(self) -> None:
        """发布一次完整桌面，随后停止定时器。"""
        table = CollisionObject()
        table.header.frame_id = self.frame_id
        table.header.stamp = self.get_clock().now().to_msg()
        table.id = "table_z0_with_base_cutout"
        table.operation = CollisionObject.ADD

        outer = self.table_size
        cutout = self.cutout_size
        side_width = (outer - cutout) / 2.0
        side_offset = (outer + cutout) / 4.0
        center_z = self.surface_z - self.thickness / 2.0

        # 左右两块覆盖完整Y方向；前后两块只覆盖中间开口宽度，
        # 四块合起来形成桌面，并在base_link原点周围留下方形开口。
        boxes = (
            (side_width, outer, -side_offset, 0.0),
            (side_width, outer, side_offset, 0.0),
            (cutout, side_width, 0.0, -side_offset),
            (cutout, side_width, 0.0, side_offset),
        )
        for size_x, size_y, center_x, center_y in boxes:
            table.primitives.append(
                self.make_box(size_x, size_y, self.thickness)
            )
            table.primitive_poses.append(
                self.make_pose(center_x, center_y, center_z)
            )

        self.publisher.publish(table)
        self.publish_timer.cancel()
        self.get_logger().info(
            "已发布桌面碰撞体：frame=%s，上表面z=%.3f m，尺寸=%.2f m，"
            "底座开口=%.2f m，厚度=%.3f m",
            self.frame_id,
            self.surface_z,
            self.table_size,
            self.cutout_size,
            self.thickness,
        )


def main(args=None) -> None:
    """启动桌面碰撞体发布节点。"""
    rclpy.init(args=args)
    node = TableCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
