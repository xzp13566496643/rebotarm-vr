import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动GenericSystem、控制器、MoveIt Servo、VR映射和RViz仿真。"""
    use_rviz = LaunchConfiguration("use_rviz")
    package_share = get_package_share_directory("rebotarm_moveit")
    config_directory = os.path.join(package_share, "config")
    xacro_path = os.path.join(
        package_share, "description", "urdf", "rebotarm.urdf.xacro"
    )
    initial_positions_path = os.path.join(
        config_directory, "initial_positions.yaml"
    )
    robot_description = {
        "robot_description": xacro.process_file(
            xacro_path,
            mappings={
                "use_fake_hardware": "true",
                "initial_positions_file": initial_positions_path,
            },
        ).toxml()
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
    with open(
        os.path.join(config_directory, "servo_hardware.yaml"),
        "r",
        encoding="utf-8",
    ) as stream:
        servo_config = yaml.safe_load(stream)
    # 仿真执行层使用ros2_control标准话题，其余Servo参数继续与真机共用。
    servo_config["joint_topic"] = "/joint_states"
    servo_config["command_out_topic"] = (
        "/rebotarm_controller/joint_trajectory"
    )
    servo_parameters = {"moveit_servo": servo_config}

    controllers_path = os.path.join(
        config_directory, "ros2_controllers.yaml"
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_path],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rebotarm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_path,
        ],
        output="screen",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_path,
        ],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    # 控制算法输入仍是同一个Twist话题；Servo输出在上面改为仿真控制器话题。
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
        ],
    )
    vr_control = Node(
        package="openarmx_teleop_bridge_vr",
        executable="vr2control_node",
        name="vr_to_control",
        output="screen",
    )
    gripper_bridge = Node(
        package="rebotarm_moveit",
        executable="sim_gripper_bridge.py",
        name="sim_gripper_bridge",
        output="screen",
        parameters=[{
            "open_motor_position": -4.6,
            "closed_motor_position": -0.1,
            "max_finger_position": 0.045,
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[robot_description],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        world_to_base,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        servo_node,
        vr_control,
        gripper_bridge,
        rviz,
    ])
