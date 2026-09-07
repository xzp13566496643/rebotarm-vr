import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """启动真机 MoveIt Servo、VR累计位姿映射和桌面碰撞体发布节点。"""
    gripper_control_mode = LaunchConfiguration("gripper_control_mode")
    gripper_mit_kp = LaunchConfiguration("gripper_mit_kp")
    gripper_mit_kd = LaunchConfiguration("gripper_mit_kd")
    gripper_opening_torque_limit = LaunchConfiguration(
        "gripper_opening_torque_limit")
    gripper_motor_vlim = LaunchConfiguration("gripper_motor_vlim")
    gripper_close_torque = LaunchConfiguration("gripper_close_torque")
    gripper_hold_torque = LaunchConfiguration("gripper_hold_torque")
    gripper_tau_max = LaunchConfiguration("gripper_tau_max")
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
            "position_gain": 3.0,
            "orientation_gain": 3.0,
            "max_linear_speed": 0.40,
            "max_angular_speed": 0.80,
            "hardware_gripper": True,
            "gripper_control_mode": ParameterValue(
                gripper_control_mode, value_type=str),
            # 避开实测约 -4.72 rad 的打开侧机械硬限位。
            "gripper_motor_open": -4.60,
            "gripper_motor_vlim": ParameterValue(
                gripper_motor_vlim, value_type=float),
            "gripper_mit_kp": ParameterValue(gripper_mit_kp, value_type=float),
            "gripper_mit_kd": ParameterValue(gripper_mit_kd, value_type=float),
            "gripper_opening_torque_limit": ParameterValue(
                gripper_opening_torque_limit, value_type=float),
            "gripper_close_torque": ParameterValue(
                gripper_close_torque, value_type=float),
            "gripper_hold_torque": ParameterValue(
                gripper_hold_torque, value_type=float),
            "gripper_tau_max": ParameterValue(gripper_tau_max, value_type=float),
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
    return LaunchDescription([
        DeclareLaunchArgument(
            "gripper_control_mode",
            default_value="pos_vel",
            description="Hardware gripper mode: pos_vel, mit or grasp_mit",
        ),
        DeclareLaunchArgument("gripper_motor_vlim", default_value="2.0"),
        DeclareLaunchArgument("gripper_mit_kp", default_value="5.0"),
        DeclareLaunchArgument("gripper_mit_kd", default_value="1.0"),
        DeclareLaunchArgument(
            "gripper_opening_torque_limit", default_value="1.0"),
        DeclareLaunchArgument("gripper_close_torque", default_value="0.5"),
        DeclareLaunchArgument("gripper_hold_torque", default_value="0.30"),
        DeclareLaunchArgument("gripper_tau_max", default_value="1.5"),
        servo,
        mapping,
        table_collision,
    ])
