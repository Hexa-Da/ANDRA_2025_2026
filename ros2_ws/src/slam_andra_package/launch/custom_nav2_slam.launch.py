from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    # Get the launch directory
    bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file to load')

    # Define the nav2 parameters file path
    nav2_params_path = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use')

    # Define the behavior tree XML file path
    bt_xml_path = os.path.join(
        FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=bt_xml_path,
        description='Full path to the behavior tree XML file')

    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_file,
            'use_lifecycle_mgr': 'false'
        }.items())

    # Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'use_lifecycle_mgr': 'false',
            'map': map_yaml_file
        }.items())

    # RViz
    rviz_config_dir = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        slam_launch,
        nav2_bringup_launch,
        rviz2_node
    ])
