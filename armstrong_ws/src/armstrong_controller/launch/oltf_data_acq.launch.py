import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file_path = os.path.join(
        get_package_share_directory('armstrong_controller'),
        'config',
        'oltf_data_acq.yaml' # Name of your YAML file
    )

    return LaunchDescription([
        Node(
            package='armstrong_controller',
            executable='oltf_data_acq.py', # Name of your Python script
            name='oltf_data_acq_node',
            output='screen',
            parameters=[config_file_path]
        ),
    ])