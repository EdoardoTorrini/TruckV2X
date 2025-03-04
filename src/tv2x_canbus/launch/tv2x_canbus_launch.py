from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('tv2x_canbus'),
        'config',
        'config.yaml'
    )

    node=Node(
        package='tv2x_canbus',
        name='tv2x_canbus_node',
        executable='tv2x_canbus_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])