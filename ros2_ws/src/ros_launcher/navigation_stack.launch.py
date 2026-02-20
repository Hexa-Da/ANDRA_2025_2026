# navigation_stack.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_amcl = LaunchConfiguration('use_amcl', default='false')
    map_path = LaunchConfiguration('map_path', default='')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true') 
    enable_scout = LaunchConfiguration('enable_scout', default='true')
    enable_zed = LaunchConfiguration('enable_zed', default='true')
    enable_ptz = LaunchConfiguration('enable_ptz', default='true')
    enable_image_transfer = LaunchConfiguration('enable_image_transfer', default='true')
    enable_video_publisher = LaunchConfiguration('enable_video_publisher', default='true')
    ptz_brightness = LaunchConfiguration('ptz_brightness', default='1.0')
    ptz_contrast = LaunchConfiguration('ptz_contrast', default='1.0')
    ptz_gamma = LaunchConfiguration('ptz_gamma', default='1.0')
    video_extract_rate = LaunchConfiguration('video_extract_rate', default='10.0')
    
    # Get package share directory for config files
    pkg_share = get_package_share_directory('ros_launcher')
    
    # Full path to configuration files
    ekf_config = os.path.join(pkg_share, 'configs', 'ekf_config.yaml')
    slam_config = os.path.join(pkg_share, 'configs', 'slam_config.yaml')
    amcl_config = os.path.join(pkg_share, 'configs', 'amcl_config.yaml')

    return LaunchDescription([
        # World frame - parent of odom (always available, even without SLAM)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen',
        ),
        
        # Launch arguments
        DeclareLaunchArgument('use_slam', default_value='true',
                             description='Use SLAM for mapping'),
        DeclareLaunchArgument('use_amcl', default_value='false',
                             description='Use AMCL for localization'),
        DeclareLaunchArgument('map_path', default_value='',
                             description='Full path to map yaml file'),
        DeclareLaunchArgument('enable_lidar', default_value='true',
                             description='Enable YDLIDAR driver'),
        DeclareLaunchArgument('enable_scout', default_value='true',
                             description='Enable Scout base driver'),
        DeclareLaunchArgument('enable_zed', default_value='true',
                             description='Enable ZED camera'),
        DeclareLaunchArgument('enable_ptz', default_value='true',
                             description='Enable PTZ camera'),
        DeclareLaunchArgument('enable_image_transfer', default_value='true',
                             description='Enable image transfer node'),
        DeclareLaunchArgument('enable_video_publisher', default_value='true',
                             description='Enable video publisher node'),
        DeclareLaunchArgument('ptz_brightness', default_value='1.0',
                             description='Brightness multiplier for PTZ images (1.0=normal, >1.0=brighter)'),
        DeclareLaunchArgument('ptz_contrast', default_value='1.0',
                             description='Contrast multiplier for PTZ images (1.0=normal, >1.0=more contrast)'),
        DeclareLaunchArgument('ptz_gamma', default_value='1.0',
                             description='Gamma correction for PTZ images (1.0=normal, <1.0=brighter)'),
        DeclareLaunchArgument('video_extract_rate', default_value='10.0',
                             description='Extract rate for video publisher'),


        # YDLIDAR - conditionnel
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'TG.yaml')],
            output='screen',
            condition=IfCondition(enable_lidar),
        ),
     
        # Scout base - conditionnel
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_base', 'scout_mini_base.launch.py', 
                 'port_name:=agilex', 'is_scout_mini:=True', 'odom_topic_name:=odom_robot'],
            output='screen',
            condition=IfCondition(enable_scout),
        ),

        # ZED Camera - conditionnel
        ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py', 'camera_model:=zed2i'],
            output='screen',
            condition=IfCondition(enable_zed),
        ),
        
        # Image transfer nodes - conditionnel
        ExecuteProcess(
            cmd=['ros2', 'run', 'image_transfer', 'position_publisher'],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_image_transfer),
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'navigation_utils', 'report_fissures'],
            output='screen',
            condition=IfCondition(enable_image_transfer),
        ),
        # Conversion odométrie vers Path pour visualisation
        Node(
            package='navigation_utils',
            executable='odom_to_path',
            name='odom_to_path',
            output='screen',
        ),

        # PTZ camera publisher - conditionnel et configurable
        Node(
            package='image_transfer',
            executable='image_publisher',
            name='image_publisher',
            parameters=[{
                'brightness': PythonExpression(['float(', ptz_brightness, ')']),
                'contrast': PythonExpression(['float(', ptz_contrast, ')']),
                'gamma': PythonExpression(['float(', ptz_gamma, ')']),
                'images_output_dir': 'ros2_ws/images_capturees',
                'enable_capture_auto': False,
            }],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_ptz),
        ),

        # Video file publisher - alternative à la PTZ (lit video/video_output/)
        Node(
            package='image_transfer',
            executable='video_publisher',
            name='video_file_publisher',
            parameters=[{
                'extract_rate': PythonExpression(['float(', video_extract_rate, ')']),
            }],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_video_publisher),
        ),
        
        # PTZ controller - conditionnel
        Node(
            package='image_transfer',
            executable='ptz_controller',
            name='ptz_controller',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_ptz),
        ),
        
        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'zed_camera_link']
        ),
       Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           name='base_to_laser_tf',
           arguments=['0', '0', '0', '1.57', '0', '0', 'base_link', 'laser_frame']
       ),
        
        # Robot localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config]
        ),
        
        # SLAM Toolbox (mapping mode)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            condition=IfCondition(use_slam)
        ),
        
        # AMCL navigation stack (localization mode)
        GroupAction([
            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_path}],
                output='screen',
                condition=IfCondition(use_amcl)
            ),
            
            # AMCL localizer
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[amcl_config],
                remappings=[
                    ('scan', 'scan'),
                    ('map', 'map'),
                    ('map_metadata', 'map_metadata')
                ],
                output='screen',
                condition=IfCondition(use_amcl)
            ),
            
            # Nav2 lifecycle manager - this is critical for AMCL and map_server
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}
                ],
                condition=IfCondition(use_amcl)
            ),
            
            # Transform publisher between map and odom frames
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_fallback',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                # This will be overridden by AMCL once it starts publishing
                condition=IfCondition(use_amcl)
            )
        ])
    ])