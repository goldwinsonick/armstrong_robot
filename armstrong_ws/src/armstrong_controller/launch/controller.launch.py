from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, NotSubstitution

def generate_launch_description():
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_wheel_controller = Node(package='controller_manager', executable='spawner',
                            arguments=[ 'joint1_effort_controller', 
                                        'joint2_effort_controller',],
                            output='screen')
    
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        spawn_wheel_controller,
    ])