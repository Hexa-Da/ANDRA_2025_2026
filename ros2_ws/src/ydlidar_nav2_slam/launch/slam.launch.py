# ~/ros2_ws/src/ydlidar_nav2_slam/launch/slam.launch.py
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
    slam_params_file = os.path.join(ydlidar_nav2_slam_dir, 'config', 'slam_params.yaml')
    
    # Include the YDLidar launch file
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ydlidar_nav2_slam_dir, 'launch', 'ydlidar.launch.py')
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
            
        # Include YDLidar launch
        ydlidar_launch,
            
        # SLAM Toolbox
        Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen')
    ])
