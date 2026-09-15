from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """启动reBotArm真机驱动并传入硬件参数。"""
    hardware_config_path = os.path.join(get_package_share_directory("rebotarm_driver")  
                                        ,"config","rebotarm_hardware.yaml")
    return LaunchDescription([
        Node(
            package="rebotarm_driver",
            executable="rebotarm_driver",
            name="rebotarm_driver",
            output="screen",
            parameters=[{
                "hardware_config": hardware_config_path,
                "sdk_root": "/home/xuzhanpeng/ws/arm_controll/driver/reBotArm_control_py",
            }],
        ),
    ])
