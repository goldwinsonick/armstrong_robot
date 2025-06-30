import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    armstrong_description_pkg_path = get_package_share_directory('armstrong_description')
    armstrong_controller_pkg_path = get_package_share_directory('armstrong_controller')

    lock_joint1_arg = DeclareLaunchArgument(
        "lock_joint1", default_value="False",
        description="Lock joint1 (Arm to base) if true"
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        "use_teleop", default_value="False",
        description="Control the Robot's Joint using keyboard teleop"
    )
    
    use_pid_arg = DeclareLaunchArgument(
        "use_pid", default_value="False",
        description="Run PID Control based on target published to a topic"
    )
    
    use_target_arg = DeclareLaunchArgument(
        "use_target", default_value="False",
        description="Run target for PID (Square Signal)"
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

    teleop_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_controller_pkg_path, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_teleop'))
    )

    pid_controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_controller_pkg_path, 'launch', 'joint_pid.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_pid'))
    )

    target_publisher_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(armstrong_controller_pkg_path, 'launch', 'joint2_target_publisher.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_target'))
    )

    return LaunchDescription([
        lock_joint1_arg,
        use_teleop_arg,
        use_pid_arg,
        use_target_arg,
        
        gazebo_launch_include,
        controller_launch_include,
        
        teleop_launch_include,
        pid_controller_launch_include,
        target_publisher_launch_include,
    ])