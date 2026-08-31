import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    config_path = os.path.join(
        get_package_share_directory("rebotarm_command"), "config", "servo_hardware.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        servo_params = {"moveit_servo": yaml.safe_load(stream)}

    servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    mapping = Node(
        package="rebotarm_command",
        executable="vr_pose_target_servo.py",
        name="vr_pose_target_servo",
        output="screen",
        parameters=[{
            "position_scale": 0.8,
            "orientation_scale": 1.0,
            "position_gain": 2.0,
            "orientation_gain": 2.0,
            "max_linear_speed": 0.10,
            "max_angular_speed": 0.60,
            "hardware_gripper": True,
            "gripper_motor_vlim": 2.0,
        }],
    )
    table_collision = Node(
        package="rebotarm_command",
        executable="table_collision.py",
        name="table_collision",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "surface_z": 0.0,
            "base_cutout_size": 0.40,
        }],
    )
    return LaunchDescription([servo, mapping, table_collision])
