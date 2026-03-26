import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    airsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('airsim_ros_pkgs'),
                'launch',
                'airsim_node.launch.py',
            )
        )
    )

    move_velocity_node = Node(
        package='hw_insight',
        executable='move_velocity',
        name='move_velocity',
        output='screen',
    )

    text_command_bridge_node = Node(
        package='hw_insight',
        executable='text_command_bridge',
        name='text_command_bridge',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(airsim_launch)
    ld.add_action(move_velocity_node)
    ld.add_action(text_command_bridge_node)
    return ld
