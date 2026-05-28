import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time  = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    nav2_params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'nav2_params.yaml'
    )

    # map_server, loads saved YAML map from SLAM session and publishes /map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # AMCL — Adaptive Monte Carlo Localisation
    # Uses particle filter to locate robot on existing map
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Lifecycle manager for localisation nodes only, Manages map_server and amcl startup in correct order
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file'
        ),
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])