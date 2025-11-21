from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    
    # Paths
    params_path = PathJoinSubstitution(
        [FindPackageShare("slam_andra_package"), 'config', 'nav2_params.yaml'])
    
    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_path,
        description='Full path to the ROS2 parameters file to use')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # Since your TF tree already shows all the necessary frames
    # we only need to launch the Nav2 stack with SLAM
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_andra_package'),
                'launch',
                'custom_nav2_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'slam': 'True'
        }.items()
    )
    
    # Robot Localization for sensor fusion (ZED odometry + IMU)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/zed/zed_node/odom',
            'odom0_config': [
                True, True, False,   # x, y, z
                False, False, True,  # roll, pitch, yaw
                True, True, False,   # vx, vy, vz
                False, False, True,  # vroll, vpitch, vyaw
                False, False, False  # ax, ay, az
            ],
            'odom0_queue_size': 10,
            'odom0_nodelay': True,
            'odom0_differential': False,
            'odom0_relative': False,
            'imu0': '/zed/zed_node/imu/data',
            'imu0_config': [
                False, False, False,  # x, y, z
                True, True, True,     # roll, pitch, yaw
                False, False, False,  # vx, vy, vz
                True, True, True,     # vroll, vpitch, vyaw
                True, True, True      # ax, ay, az
            ],
            'imu0_nodelay': True,
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_queue_size': 10,
            'imu0_remove_gravitational_acceleration': True
        }]
    )
    
    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        nav2_slam_launch,
        robot_localization_node
    ])
