import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your YAML config file
    param_file = os.path.join(
        get_package_share_directory('armstrong_controller'),  # <-- change to your package name
        'config',
        'joint2_target_publisher.yaml'  # <-- change to your YAML file name
    )

    return LaunchDescription([
        Node(
            package='armstrong_controller',
            executable='joint2_target_publisher.py',
            name='joint2_target_publisher',
            output='screen',
            parameters=[param_file]
        ),
    ])