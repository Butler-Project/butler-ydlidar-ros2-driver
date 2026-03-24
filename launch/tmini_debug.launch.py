import os
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmini_driver_node')
    params_file = os.path.join(pkg_dir, 'params', 'tmini.yaml')

    lidar_node = Node(
        package='tmini_driver_node',
        executable='tmini_driver_node',
        name='tmini_driver_node',
        output='screen',
        parameters=[params_file],
    )

    foxglove_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            FindPackageShare('foxglove_bridge'),
            '/launch/foxglove_bridge_launch.xml'
        ])
    )

    return LaunchDescription([
        lidar_node,
        foxglove_bridge,
    ])
