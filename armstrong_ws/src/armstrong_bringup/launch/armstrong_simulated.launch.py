import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    armstrong_description_pkg_path = get_package_share_directory('armstrong_description')
    armstrong_controller_pkg_path = get_package_share_directory('armstrong_controller')

    # Parameters
    lock_joint1_arg = DeclareLaunchArgument(
        "lock_joint1", default_value="False",
        description="Lock joint1"
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_description_pkg_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'lock_joint1': LaunchConfiguration('lock_joint1')}.items()
    )

    controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_controller_pkg_path, 'launch', 'controller.launch.py')
        )
    )

    return LaunchDescription([
        lock_joint1_arg,
        gazebo_launch_include,
        controller_launch_include,
    ])