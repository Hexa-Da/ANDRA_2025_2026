# navigation_stack.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_launcher')
    scout_urdf = os.path.join(
        get_package_share_directory('scout_description'),
        'urdf', 'scout_mini', 'scout_mini.urdf'
    )
    ekf_config = os.path.join(pkg_share, 'configs', 'ekf_config.yaml')
    slam_config = os.path.join(pkg_share, 'configs', 'slam_config.yaml')
    amcl_config = os.path.join(pkg_share, 'configs', 'amcl_config.yaml')
    with open(scout_urdf, 'r', encoding='utf-8') as f:
        scout_robot_description = f.read()

    # --- Declare all launch arguments FIRST ---
    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Use SLAM for mapping')
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl', default_value='false',
        description='Use AMCL for localization')
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value='',
        description='Full path to map yaml file')
    declare_enable_lidar = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Enable YDLIDAR driver')
    declare_enable_scout = DeclareLaunchArgument(
        'enable_scout', default_value='true',
        description='Enable Scout base driver')
    declare_enable_zed = DeclareLaunchArgument(
        'enable_zed', default_value='true',
        description='Enable ZED camera')
    declare_enable_ptz = DeclareLaunchArgument(
        'enable_ptz', default_value='true',
        description='Enable PTZ camera')
    declare_enable_image_transfer = DeclareLaunchArgument(
        'enable_image_transfer', default_value='true',
        description='Enable image transfer node')
    declare_enable_video_publisher = DeclareLaunchArgument(
        'enable_video_publisher', default_value='true',
        description='Enable video publisher node')
    declare_ptz_brightness = DeclareLaunchArgument(
        'ptz_brightness', default_value='1.0',
        description='Brightness multiplier for PTZ images (1.0=normal, >1.0=brighter)')
    declare_ptz_contrast = DeclareLaunchArgument(
        'ptz_contrast', default_value='1.0',
        description='Contrast multiplier for PTZ images (1.0=normal, >1.0=more contrast)')
    declare_ptz_gamma = DeclareLaunchArgument(
        'ptz_gamma', default_value='1.0',
        description='Gamma correction for PTZ images (1.0=normal, <1.0=brighter)')
    declare_video_extract_rate = DeclareLaunchArgument(
        'video_extract_rate', default_value='10.0',
        description='Extract rate for video publisher')
    declare_ydlidar_params_file = DeclareLaunchArgument(
        'ydlidar_params_file',
        default_value='ydlidar_TG15.yaml',
        description='Fichier YAML dans ros_launcher/configs pour le YDLidar TG15')

    # --- Resolve launch configurations AFTER declarations ---
    use_slam = LaunchConfiguration('use_slam')
    use_amcl = LaunchConfiguration('use_amcl')
    map_path = LaunchConfiguration('map_path')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_scout = LaunchConfiguration('enable_scout')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_ptz = LaunchConfiguration('enable_ptz')
    enable_image_transfer = LaunchConfiguration('enable_image_transfer')
    enable_video_publisher = LaunchConfiguration('enable_video_publisher')
    ptz_brightness = LaunchConfiguration('ptz_brightness')
    ptz_contrast = LaunchConfiguration('ptz_contrast')
    ptz_gamma = LaunchConfiguration('ptz_gamma')
    video_extract_rate = LaunchConfiguration('video_extract_rate')
    ydlidar_params_file = LaunchConfiguration('ydlidar_params_file')

    return LaunchDescription([
        # 1. Declarations 
        declare_use_slam,
        declare_use_amcl,
        declare_map_path,
        declare_enable_lidar,
        declare_enable_scout,
        declare_enable_zed,
        declare_enable_ptz,
        declare_enable_image_transfer,
        declare_enable_video_publisher,
        declare_ptz_brightness,
        declare_ptz_contrast,
        declare_ptz_gamma,
        declare_video_extract_rate,
        declare_ydlidar_params_file,

        # 2. Drivers matériels (conditionnels)

        # YDLIDAR
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[
                ParameterFile(
                    PathJoinSubstitution([
                        FindPackageShare('ros_launcher'),
                        'configs',
                        ydlidar_params_file,
                    ]),
                    allow_substs=False,
                ),
            ],
            output='screen',
            condition=IfCondition(enable_lidar),
        ),

        # Scout base
        ExecuteProcess(
            cmd=['ros2', 'launch', 'scout_base', 'scout_mini_base.launch.py',
                 'port_name:=agilex', 'is_scout_mini:=True', 'odom_topic_name:=odom_robot'],
            output='screen',
            condition=IfCondition(enable_scout),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': scout_robot_description}],
            output='screen',
            condition=IfCondition(enable_scout),
        ),

        # ZED Camera
        ExecuteProcess(
            # depth_mode:=NONE disables depth processing/publication while keeping RGB and IMU streams.
            cmd=[
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                'camera_model:=zed2i',
                'depth_mode:=NONE',
                'publish_tf:=false',
                'publish_map_tf:=false',
            ],
            output='screen',
            condition=IfCondition(enable_zed),
        ),

        # 4. Nœuds applicatifs (conditionnels)

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

        # PTZ camera publisher
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

        # Video file publisher
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

        # PTZ controller
        Node(
            package='image_transfer',
            executable='ptz_controller',
            name='ptz_controller',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_ptz),
        ),

        # 5. Static transforms (capteurs)

        # base_link -> zed_camera_link : x ≈ 18,5 cm, z ≈ 19,2 cm
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed_tf',
            arguments=[
                '0.185', '0.0', '0.192',
                '0.0', '0.0', '0.0',
                'base_link', 'zed_camera_link',
            ],
        ),
        # base_link -> laser_frame : x ≈ 8,5 cm, z ≈ 22,2 cm ; pas de rotation (yaw pitch roll = 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.085', '0.0', '0.222', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        # 6. Localisation et cartographie

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config]
        ),

        # SLAM Toolbox (mode cartographie)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            condition=IfCondition(use_slam)
        ),

        # AMCL (mode localisation avec carte existante)
        GroupAction([
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_path}],
                output='screen',
                condition=IfCondition(use_amcl)
            ),
            
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
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_fallback',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                condition=IfCondition(use_amcl)
            )
        ])
    ])
