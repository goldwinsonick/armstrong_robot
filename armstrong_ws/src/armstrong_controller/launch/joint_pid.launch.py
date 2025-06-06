import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pid_config_file_path = os.path.join(
        get_package_share_directory('armstrong_controller'),
        'config',
        'joint_pid_params.yaml' # Name of your PID YAML file
    )

    return LaunchDescription([
        Node(
            package='armstrong_controller',
            executable='joint_pid_controller', # Name of your C++ executable
            name='joint_pid_controller_node',
            output='screen',
            parameters=[pid_config_file_path]
        ),
    ])