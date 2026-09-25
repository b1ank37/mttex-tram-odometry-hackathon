import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('tram_reserve_odometry'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='tram_reserve_odometry',
            executable='reserve_odometry_node',
            name='reserve_odometry_node',
            output='screen',
            parameters=[params_file],
        )
    ])