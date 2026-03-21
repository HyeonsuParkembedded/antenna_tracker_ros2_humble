import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('antenna_tracker_simulation')
    pkg_controller = get_package_share_directory('antenna_tracker_controller')

    xacro_file = os.path.join(pkg_sim, 'urdf', 'antenna_tracker.urdf.xacro')
    world_file = os.path.join(pkg_sim, 'worlds', 'antenna_tracker.world')
    rviz_config = os.path.join(pkg_sim, 'config', 'rviz_config.rviz')

    robot_description = Command(['xacro ', xacro_file])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'antenna_tracker'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),

        # Sensor Fusion Node
        Node(
            package='antenna_tracker_controller',
            executable='sensor_fusion_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Navigation Node
        Node(
            package='antenna_tracker_controller',
            executable='navigation_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Controller Node
        Node(
            package='antenna_tracker_controller',
            executable='controller_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # State Machine Node
        Node(
            package='antenna_tracker_controller',
            executable='state_machine_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
