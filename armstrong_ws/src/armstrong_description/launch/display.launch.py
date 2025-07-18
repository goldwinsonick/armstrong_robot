import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory('armstrong_description'), 'urdf', 'armstrong_v1', 'main.urdf.xacro'),
        description="Path to the robot model file",
    )
    
    robot_description =  ParameterValue(Command(['xacro ', LaunchConfiguration('model')]))
    
    # Robot state publisher (publishes /tf based on urdf and joint states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('armstrong_description'), 'rviz', 'armstrong_display.rviz')],
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])