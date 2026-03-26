from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_node = Node(
        package='hw_insight',
        executable='gcs_dashboard',
        name='gcs_dashboard',
        output='screen',
        parameters=[
            {'refresh_rate_hz': 4.0},
            {'event_rows': 12},
            {'clear_screen': True},
        ],
    )

    return LaunchDescription([dashboard_node])
