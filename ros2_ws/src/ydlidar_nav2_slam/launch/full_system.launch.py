# ~/ros2_ws/src/ydlidar_nav2_slam/launch/full_system.launch.py
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
    
    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ydlidar_nav2_slam_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include the Navigation launch file
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ydlidar_nav2_slam_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        # Include SLAM launch
        slam_launch,
            
        # Include Navigation launch
        nav_launch,
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
            output='screen')
    ])
