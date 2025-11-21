# ~/ros2_ws/src/ydlidar_nav2_slam/launch/navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    ydlidar_nav2_slam_dir = get_package_share_directory('ydlidar_nav2_slam')
    nav2_params_file = os.path.join(ydlidar_nav2_slam_dir, 'config', 'nav2_params.yaml')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        # Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file
            }.items()
        )
    ])
