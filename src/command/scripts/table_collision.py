#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


class TableCollisionPublisher(Node):
    """Publish a z=0 tabletop while leaving clearance for the fixed robot base."""

    def __init__(self) -> None:
        super().__init__("table_collision")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("table_size", 4.0)
        self.declare_parameter("base_cutout_size", 0.40)
        self.declare_parameter("thickness", 0.04)
        self.declare_parameter("surface_z", 0.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            CollisionObject, "/collision_object", qos)
        self.timer = self.create_timer(1.0, self.publish_table)
        self.published_once = False

    @staticmethod
    def add_box(msg: CollisionObject, dimensions, xyz) -> None:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        msg.primitives.append(primitive)
        msg.primitive_poses.append(pose)

    def publish_table(self) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        size = float(self.get_parameter("table_size").value)
        cutout = float(self.get_parameter("base_cutout_size").value)
        thickness = float(self.get_parameter("thickness").value)
        surface_z = float(self.get_parameter("surface_z").value)

        if not 0.0 < cutout < size or thickness <= 0.0:
            self.get_logger().error("Invalid table dimensions")
            return

        side_width = (size - cutout) / 2.0
        side_center = (size + cutout) / 4.0
        z_center = surface_z - thickness / 2.0

        msg = CollisionObject()
        msg.header.frame_id = frame_id
        msg.id = "table_z0_with_base_cutout"
        msg.operation = CollisionObject.ADD

        # Two full-length side slabs plus two center slabs form a tabletop with
        # a square opening around the fixed base, avoiding permanent contact.
        self.add_box(msg, (side_width, size, thickness),
                     (-side_center, 0.0, z_center))
        self.add_box(msg, (side_width, size, thickness),
                     (side_center, 0.0, z_center))
        self.add_box(msg, (cutout, side_width, thickness),
                     (0.0, -side_center, z_center))
        self.add_box(msg, (cutout, side_width, thickness),
                     (0.0, side_center, z_center))

        self.publisher.publish(msg)
        if not self.published_once:
            self.get_logger().info(
                f"Published table collision: {frame_id} z={surface_z:.3f} m, "
                f"base cutout={cutout:.3f} m")
            self.published_once = True


def main(args=None) -> None:
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
