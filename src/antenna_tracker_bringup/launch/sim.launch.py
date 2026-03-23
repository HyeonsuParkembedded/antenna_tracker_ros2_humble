import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory('antenna_tracker_simulation')
    pkg_bringup = get_package_share_directory('antenna_tracker_bringup')
    launch_rviz = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        # Gazebo + ROS nodes (controller, navigation, state machine, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'sim.launch.py')
            ),
            launch_arguments={
                'launch_rviz': launch_rviz,
            }.items()
        ),
        # rosbridge WebSocket + HTTP GCS dashboard
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'web.launch.py')
            )
        ),
    ])
