import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动真机MoveIt Servo和C++ VR累计位姿控制节点。"""
    position_scale = LaunchConfiguration("position_scale")
    orientation_scale = LaunchConfiguration("orientation_scale")
    position_gain = LaunchConfiguration("position_gain")
    orientation_gain = LaunchConfiguration("orientation_gain")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    input_timeout = LaunchConfiguration("input_timeout")
    butterworth_filter_coeff = LaunchConfiguration("butterworth_filter_coeff")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_closed_position = LaunchConfiguration("gripper_closed_position")
    gripper_torque_max = LaunchConfiguration("gripper_torque_max")
    gripper_close_torque = LaunchConfiguration("gripper_close_torque")
    gripper_hold_torque = LaunchConfiguration("gripper_hold_torque")
    gripper_move_kp = LaunchConfiguration("gripper_move_kp")
    gripper_move_kd = LaunchConfiguration("gripper_move_kd")
    gripper_close_kp = LaunchConfiguration("gripper_close_kp")
    gripper_close_kd = LaunchConfiguration("gripper_close_kd")
    gripper_contact_torque_threshold = LaunchConfiguration(
        "gripper_contact_torque_threshold"
    )
    gripper_stall_velocity = LaunchConfiguration("gripper_stall_velocity")
    gripper_startup_distance = LaunchConfiguration("gripper_startup_distance")
    table_surface_z = LaunchConfiguration("table_surface_z")
    table_size = LaunchConfiguration("table_size")
    table_base_cutout_size = LaunchConfiguration("table_base_cutout_size")
    table_thickness = LaunchConfiguration("table_thickness")

    # 直接加载Servo真正需要的模型参数，避免依赖完整路径规划配置。
    package_share = get_package_share_directory("rebotarm_moveit")
    config_directory = os.path.join(package_share, "config")
    xacro_path = os.path.join(
        package_share, "description", "urdf", "rebotarm.urdf.xacro"
    )
    robot_description = {
        "robot_description": xacro.process_file(xacro_path).toxml()
    }
    with open(
        os.path.join(config_directory, "rebotarm.srdf"),
        "r",
        encoding="utf-8",
    ) as stream:
        robot_description_semantic = {
            "robot_description_semantic": stream.read()
        }
    with open(
        os.path.join(config_directory, "kinematics.yaml"),
        "r",
        encoding="utf-8",
    ) as stream:
        robot_description_kinematics = {
            "robot_description_kinematics": yaml.safe_load(stream)
        }
    with open(
        os.path.join(config_directory, "joint_limits.yaml"),
        "r",
        encoding="utf-8",
    ) as stream:
        robot_description_planning = {
            "robot_description_planning": yaml.safe_load(stream)
        }

    servo_config_path = os.path.join(config_directory, "servo_hardware.yaml")
    with open(servo_config_path, "r", encoding="utf-8") as stream:
        servo_parameters = {"moveit_servo": yaml.safe_load(stream)}

    # 接收/servo_node/delta_twist_cmds，输出/rebotarm/servo_joint_trajectory。
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_parameters,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "butterworth_filter_coeff": ParameterValue(
                    butterworth_filter_coeff, value_type=float
                )
            },
        ],
    )

    # 向MoveIt PlanningScene发布桌面，使Servo的碰撞检测阻止机械臂越过z=0。
    table_collision_node = Node(
        package="rebotarm_moveit",
        executable="table_collision_publisher.py",
        name="table_collision_publisher",
        output="screen",
        parameters=[{
            "table_surface_z": ParameterValue(
                table_surface_z, value_type=float
            ),
            "table_size": ParameterValue(table_size, value_type=float),
            "table_base_cutout_size": ParameterValue(
                table_base_cutout_size, value_type=float
            ),
            "table_thickness": ParameterValue(
                table_thickness, value_type=float
            ),
        }],
    )

    # 接收PICO话题，生成累计TCP目标和Twist；夹爪发布MIT恒力开合命令。
    vr_control_node = Node(
        package="openarmx_teleop_bridge_vr",
        executable="vr2control_node",
        name="vr_to_control",
        output="screen",
        parameters=[{
            "position_scale": ParameterValue(position_scale, value_type=float),
            "orientation_scale": ParameterValue(
                orientation_scale, value_type=float
            ),
            "position_gain": ParameterValue(position_gain, value_type=float),
            "orientation_gain": ParameterValue(
                orientation_gain, value_type=float
            ),
            "max_linear_speed": ParameterValue(
                max_linear_speed, value_type=float
            ),
            "max_angular_speed": ParameterValue(
                max_angular_speed, value_type=float
            ),
            "input_timeout": ParameterValue(input_timeout, value_type=float),
            "gripper_open_position": ParameterValue(
                gripper_open_position, value_type=float
            ),
            "gripper_closed_position": ParameterValue(
                gripper_closed_position, value_type=float
            ),
            "gripper_torque_max": ParameterValue(
                gripper_torque_max, value_type=float
            ),
            "gripper_close_torque": ParameterValue(
                gripper_close_torque, value_type=float
            ),
            "gripper_hold_torque": ParameterValue(
                gripper_hold_torque, value_type=float
            ),
            "gripper_move_kp": ParameterValue(
                gripper_move_kp, value_type=float
            ),
            "gripper_move_kd": ParameterValue(
                gripper_move_kd, value_type=float
            ),
            "gripper_close_kp": ParameterValue(
                gripper_close_kp, value_type=float
            ),
            "gripper_close_kd": ParameterValue(
                gripper_close_kd, value_type=float
            ),
            "gripper_contact_torque_threshold": ParameterValue(
                gripper_contact_torque_threshold, value_type=float
            ),
            "gripper_stall_velocity": ParameterValue(
                gripper_stall_velocity, value_type=float
            ),
            "gripper_startup_distance": ParameterValue(
                gripper_startup_distance, value_type=float
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("position_scale", default_value="1.2"),
        DeclareLaunchArgument("orientation_scale", default_value="1.0"),
        DeclareLaunchArgument("position_gain", default_value="30.0"),
        DeclareLaunchArgument("orientation_gain", default_value="50.0"),
        DeclareLaunchArgument("max_linear_speed", default_value="5.0"),
        DeclareLaunchArgument("max_angular_speed", default_value="10.0"),
        DeclareLaunchArgument("input_timeout", default_value="0.2"),
        # 数值越大，Servo关节位置输出越平滑，但相位滞后也越明显。
        DeclareLaunchArgument("butterworth_filter_coeff", default_value="3.0"),
        # -4.60 rad用于避开实测约-4.72 rad的打开侧机械硬限位。
        DeclareLaunchArgument("gripper_open_position", default_value="-4.60"),
        DeclareLaunchArgument("gripper_closed_position", default_value="0.0"),
        # 同学实测参数：1.50 Nm闭合，检测接触后以0.50 Nm保持。
        DeclareLaunchArgument("gripper_torque_max", default_value="1.5"),
        DeclareLaunchArgument("gripper_close_torque", default_value="1.50"),
        DeclareLaunchArgument("gripper_hold_torque", default_value="0.50"),
        DeclareLaunchArgument("gripper_move_kp", default_value="1.0"),
        DeclareLaunchArgument("gripper_move_kd", default_value="1.0"),
        DeclareLaunchArgument("gripper_close_kp", default_value="0.0"),
        DeclareLaunchArgument("gripper_close_kd", default_value="0.5"),
        DeclareLaunchArgument(
            "gripper_contact_torque_threshold", default_value="0.80"
        ),
        DeclareLaunchArgument("gripper_stall_velocity", default_value="0.015"),
        DeclareLaunchArgument("gripper_startup_distance", default_value="0.60"),
        DeclareLaunchArgument("table_surface_z", default_value="0.0"),
        DeclareLaunchArgument("table_size", default_value="4.0"),
        DeclareLaunchArgument("table_base_cutout_size", default_value="0.40"),
        DeclareLaunchArgument("table_thickness", default_value="0.04"),
        servo_node,
        table_collision_node,
        vr_control_node,
    ])
