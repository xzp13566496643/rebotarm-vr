from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "rebotarm", package_name="rebotarm_moveit_config"
        )
        .robot_description(file_path="config/rebotarm.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    diagnostic = Node(
        package="rebotarm_command",
        executable="nearest_self_distance",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([diagnostic])
