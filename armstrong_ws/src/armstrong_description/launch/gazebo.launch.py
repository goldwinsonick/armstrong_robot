import os

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty_world.world",
        description="World path"
    )
    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial x position"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Initial y position"
    )
    z_arg = DeclareLaunchArgument(
        "z", default_value="1.0", description="Initial z position"
    )
    lock_joint1_arg = DeclareLaunchArgument(
        "lock_joint1", default_value="false",
        description="Lock joint1",
    )

    # Paths
    pkg_desc = get_package_share_directory("armstrong_description")
    world_path = PathJoinSubstitution([
        pkg_desc,
        "worlds",
        LaunchConfiguration("world"),
    ])
    xacro_path = os.path.join(pkg_desc, "urdf", "armstrong_v1", "main.urdf.xacro")

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=":".join([
            str(Path(pkg_desc).parent.resolve())
        ])
    )

    # robot_description
    robot_description_command = [
        "xacro ",
        xacro_path,
        " lock_joint1_arg:=",
        LaunchConfiguration("lock_joint1"),
    ]
    robot_description = Command(robot_description_command)

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # Launch Gazebo Ignition (Gazebo Sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': [world_path, ' -r', ' -v 4']}.items(),
    )

    # Spawn robot in Gazebo Ignition
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robot",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen"
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        lock_joint1_arg,
        ign_resource_path,
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
    ])