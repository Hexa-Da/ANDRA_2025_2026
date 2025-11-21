import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Run the driver node
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py'],
            output='screen',
        ),

         # Include the ZED2 camera launch file with the camera_model argument set to 'zed2'
        ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py', 'camera_model:=zed2'],
            output='screen',
        ),
        
        # Optionally, log a message once launched
        LogInfo(
            msg="YDLidar Driver, /scan topic echoing, and ZED2 Camera are now running!"
        ),
    ])



