# ~/ros2_ws/src/ydlidar_nav2_slam/ydlidar_nav2_slam/scripts/map_saver.py
#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
import datetime

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.client = self.create_client(SaveMap, 'map_saver/save_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_saver service...')
        
        # Get home directory
        home_dir = os.path.expanduser('~')
        # Create timestamp for unique filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Create map path
        map_path = os.path.join(home_dir, f'map_{timestamp}')
        
        self.send_request(map_path)

    def send_request(self, map_path):
        request = SaveMap.Request()
        request.map_url = map_path
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            self.get_logger().info(f'Map saved to {map_path}')
        else:
            self.get_logger().error('Failed to save map')

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
