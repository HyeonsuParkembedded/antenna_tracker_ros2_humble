import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('antenna_tracker_bringup')
    params_file = os.path.join(pkg_bringup, 'config', 'hardware_params.yaml')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    return LaunchDescription([
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0',
                              description='micro-ROS agent USB port'),

        # micro-ROS Agent (USB CDC transport)
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', usb_port, '-b', '115200'],
            output='screen'
        ),

        # CAN Bridge Node (ESP32 LoRa -> TargetGPS)
        Node(
            package='antenna_tracker_hardware',
            executable='can_bridge_node',
            name='can_bridge_node',
            parameters=[params_file],
            output='screen'
        ),

        # Sensor Fusion Node
        Node(
            package='antenna_tracker_controller',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            parameters=[params_file],
            output='screen'
        ),

        # Navigation Node
        Node(
            package='antenna_tracker_controller',
            executable='navigation_node',
            name='navigation_node',
            parameters=[params_file],
            output='screen'
        ),

        # Controller Node
        Node(
            package='antenna_tracker_controller',
            executable='controller_node',
            name='controller_node',
            parameters=[params_file],
            output='screen'
        ),

        # State Machine Node
        Node(
            package='antenna_tracker_controller',
            executable='state_machine_node',
            name='state_machine_node',
            parameters=[params_file],
            output='screen'
        ),
    ])
