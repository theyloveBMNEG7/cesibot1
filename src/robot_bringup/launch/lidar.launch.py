import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        rplidar_node,
    ])