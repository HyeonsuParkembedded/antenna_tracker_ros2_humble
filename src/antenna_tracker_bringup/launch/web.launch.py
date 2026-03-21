"""
Rosbridge WebSocket launch file for the Antenna Tracker GCS Dashboard.

Usage:
  ros2 launch antenna_tracker_bringup web.launch.py

Then open http://<RPi4B-IP>:8080 (the web dashboard index.html).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_web = get_package_share_directory('antenna_tracker_web')

    port = LaunchConfiguration('port', default='9090')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090',
                              description='rosbridge WebSocket port'),

        # rosbridge WebSocket server (connects dashboard to ROS 2 backend)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': port,
                'address': '',
                'ssl': False,
                'authenticate': False,
            }],
            output='screen'
        ),

        # Simple Python HTTP server to serve static files
        Node(
            package='antenna_tracker_web',
            executable='web_server_node',
            name='gcs_web_server',
            parameters=[{
                'web_root': os.path.join(pkg_web, 'web'),
                'port': 8080,
            }],
            output='screen'
        ),
    ])
