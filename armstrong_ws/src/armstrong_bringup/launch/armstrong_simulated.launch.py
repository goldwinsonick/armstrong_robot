import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# No need to import DeclareLaunchArgument or LaunchConfiguration if not used here

def generate_launch_description():

    armstrong_description_pkg_path = get_package_share_directory('armstrong_description')
    armstrong_controller_pkg_path = get_package_share_directory('armstrong_controller')

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_description_pkg_path, 'launch', 'gazebo.launch.py')
        )
    )

    controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_controller_pkg_path, 'launch', 'controller.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch_include,
        controller_launch_include,
    ])