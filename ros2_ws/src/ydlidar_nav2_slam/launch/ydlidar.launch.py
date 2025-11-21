# ~/ros2_ws/src/ydlidar_nav2_slam/launch/ydlidar.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    ydlidar_nav2_slam_dir = get_package_share_directory('ydlidar_nav2_slam')
    ydlidar_params_file = os.path.join(ydlidar_nav2_slam_dir, 'config', 'ydlidar_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # YDLidar driver
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[ydlidar_params_file],
            output='screen'),
            
        # TF publisher for LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_publisher_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'),
            
        # Base link to odom transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_publisher_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen')
    ])
