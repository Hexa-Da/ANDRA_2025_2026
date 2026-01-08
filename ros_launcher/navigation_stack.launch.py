# navigation_stack.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
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
    ptz_ip = LaunchConfiguration('ptz_ip', default='192.168.5.163')
    ptz_port = LaunchConfiguration('ptz_port', default='554')
    ptz_brightness = LaunchConfiguration('ptz_brightness', default='1.5')
    ptz_contrast = LaunchConfiguration('ptz_contrast', default='1.25')
    ptz_gamma = LaunchConfiguration('ptz_gamma', default='1.0')
    
    # Config directory - replace 'your_package_name' with your actual package name
    # where you store your configuration files
    config_dir = 'configs'
    
    # Full path to configuration files
    ekf_config = os.path.join(config_dir, 'ekf_config.yaml')
    slam_config = os.path.join(config_dir, 'slam_config.yaml')
    amcl_config = os.path.join(config_dir, 'amcl_config.yaml')
    
   # YDLidar configuration 
    ydlidar_params_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params'
    )
    # Tester avec différents modèles : G2.yaml, G4.yaml, X4.yaml, etc.
    ydlidar_config = os.path.join(ydlidar_params_dir, 'G4.yaml')  # Commencer par G2

    return LaunchDescription([
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
        DeclareLaunchArgument('ptz_ip', default_value='192.168.5.163',
                             description='IP address of PTZ camera'),
        DeclareLaunchArgument('ptz_port', default_value='554',
                             description='RTSP port of PTZ camera'),
        DeclareLaunchArgument('ptz_brightness', default_value='1.5',
                             description='Brightness multiplier for PTZ images (1.0=normal, >1.0=brighter)'),
        DeclareLaunchArgument('ptz_contrast', default_value='1.25',
                             description='Contrast multiplier for PTZ images (1.0=normal, >1.0=more contrast)'),
        DeclareLaunchArgument('ptz_gamma', default_value='1.0',
                             description='Gamma correction for PTZ images (1.0=normal, <1.0=brighter)'),
        
        # Hardware drivers
        # YDLIDAR - conditionnel
        Node(
    	    package='ydlidar_ros2_driver',
    	    executable='ydlidar_ros2_driver_node',
    	    name='ydlidar_ros2_driver_node',
    	    parameters=[ydlidar_config, {'port': '/dev/ttyTHS1', 'baudrate': 230400}],
            output='screen',
            condition=IfCondition(enable_lidar),
        ),
        
        # Scout base driver avec interface CAN 'agilex' (configuration TechLab)
        # Utiliser le launch file officiel de scout_base pour éviter les deadlocks
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
        
        # Image transfer nodes
        ExecuteProcess(
            cmd=['ros2', 'run', 'image_transfer', 'image_subscriber'],
            output='screen',
        ),
        # PTZ camera publisher - conditionnel et configurable
        Node(
            package='image_transfer',
            executable='image_publisher',
            name='image_publisher',
            parameters=[{
                'enable_ptz': PythonExpression(["'", enable_ptz, "' == 'true'"]),
                'ptz_ip': ptz_ip,
                'ptz_port': PythonExpression(['int(', ptz_port, ')']),
                'brightness': PythonExpression(['float(', ptz_brightness, ')']),
                'contrast': PythonExpression(['float(', ptz_contrast, ')']),
                'gamma': PythonExpression(['float(', ptz_gamma, ')']),
                'enable_image_adjustment': PythonExpression(["'", enable_ptz, "' == 'true'"]),
                'save_all_images': PythonExpression(["'", enable_ptz, "' == 'true'"]),
                'images_output_dir': 'ros2_ws/images_capturees',
            }],
            output='screen',
            condition=IfCondition(enable_ptz),
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'image_transfer', 'position_publisher'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'image_transfer', 'report_fissures'],
            output='screen',
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
