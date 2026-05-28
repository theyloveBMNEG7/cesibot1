import os  
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
     # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam = LaunchConfiguration('slam')
    
    # Packages paths
    pkg_amr_robot = get_package_share_directory('amr_robot')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    # EKF config path
    ekf_config = os.path.join(pkg_robot_bringup, 'config', 'ekf.yaml')
    
    # Robot Description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_robot, 'launch', 'description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Motor Control Node
    diff_drive_node = Node(
        package='robot_drive',
        executable='diff_drive_node',
        name='diff_drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # micro-ROS Agent
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/teensy', '-v4', '--ros-args', '-p', 'use_sim_time:=false']
    )
    
    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
            {'buffer_duration': 30.0}
        ]
    )
    
    # Lidar 
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # IR sensor Node
    ir_sensor_node = Node(
        package= 'robot_drive',
        executable='ir_sensor_node',
        name='ir_sensor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # use it when we need to do autonomous navigation with odometry and IMU only.
    map_to_odom = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_odom',
      output="screen",
      arguments= ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
      condition= UnlessCondition(slam)
    )
    
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true for Gazebo'
        ), 
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='set to false if you want to use Odometry only for navigation'
        ), 
        
        robot_description,
        diff_drive_node,
        micro_ros_agent,
        ir_sensor_node,
        lidar, 
        ekf_node,
        map_to_odom,
    ])
    
    