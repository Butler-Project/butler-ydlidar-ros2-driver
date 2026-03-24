import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmini_driver_node')
    params_file = os.path.join(pkg_dir, 'params', 'tmini.yaml')

    return LaunchDescription([
        Node(
            package='tmini_driver_node',
            executable='tmini_driver_node',
            name='tmini_driver_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
