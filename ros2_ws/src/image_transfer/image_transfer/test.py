#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import time
import yaml
import argparse
import os
import asyncio

class MapChecker(Node):
    def __init__(self, map_yaml_path=None, service_timeout=2.0):
        super().__init__('map_checker')
        
        # Set a timeout for the service call
        self.service_timeout = service_timeout
        
        # Check if map YAML exists and is correctly formatted
        if map_yaml_path:
            self.check_map_yaml(map_yaml_path)
        
        # Create a subscriber for the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            1  # QoS profile, very small queue
        )
        
        # Also try the debug map topic
        self.debug_map_sub = self.create_subscription(
            OccupancyGrid,
            'debug_map',
            self.debug_map_callback,
            1
        )
        
        # Create a client for the map service
        self.map_client = self.create_client(GetMap, 'map_server/map')
        
        # Track our findings
        self.got_map_from_topic = False
        self.got_map_from_debug_topic = False
        self.got_map_from_service = False
        
        self.get_logger().info('Map checker initialized. Waiting for map data...')
        
        # Print topic info to help diagnose QoS issues
        self.get_logger().info('Checking topic publishers...')
        self.create_timer(0.5, self.check_topic_info)
        
        # Set up a timer to check the service after a brief delay
        self.service_timer = self.create_timer(2.0, self.service_timer_callback)
    
    def check_topic_info(self):
        try:
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'info', '/map'], 
                                   capture_output=True, text=True)
            self.get_logger().info(f"Map topic info:\n{result.stdout}")
            
            # Check services too
            service_result = subprocess.run(['ros2', 'service', 'list'], 
                                           capture_output=True, text=True)
            self.get_logger().info(f"Available services:\n{service_result.stdout}")
            
            # Cancel this timer after first run
            self.destroy_timer(self.executor.get_timer(self.check_topic_info))
        except Exception as e:
            self.get_logger().error(f"Error checking topic info: {str(e)}")
    
    def check_map_yaml(self, map_yaml_path):
        try:
            if not os.path.exists(map_yaml_path):
                self.get_logger().error(f"Map YAML file not found: {map_yaml_path}")
                return
            
            with open(map_yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
            
            # Check required fields
            required_fields = ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh']
            for field in required_fields:
                if field not in map_data:
                    self.get_logger().error(f"Map YAML is missing required field: {field}")
                    return
            
            # Check image path
            image_path = map_data['image']
            if not os.path.isabs(image_path):
                # If relative path, check relative to YAML
                full_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
                if not os.path.exists(full_path):
                    self.get_logger().error(f"Map image file not found: {full_path}")
                    return
            elif not os.path.exists(image_path):
                self.get_logger().error(f"Map image file not found: {image_path}")
                return
            
            self.get_logger().info("Map YAML file is valid.")
            self.get_logger().info(f"Image: {map_data['image']}")
            self.get_logger().info(f"Resolution: {map_data['resolution']}")
            self.get_logger().info(f"Origin: {map_data['origin']}")
            
        except Exception as e:
            self.get_logger().error(f"Error checking map YAML: {str(e)}")
    
    def map_callback(self, msg):
        if not self.got_map_from_topic:
            self.got_map_from_topic = True
            self.get_logger().info('Received map data from topic!')
            self.get_logger().info(f"Map frame_id: {msg.header.frame_id}")
            self.get_logger().info(f"Map dimensions: {msg.info.width}x{msg.info.height} cells")
            self.get_logger().info(f"Map resolution: {msg.info.resolution} meters/cell")
            
            # Check if map has actual data
            non_zero = sum(1 for cell in msg.data if cell != -1)
            self.get_logger().info(f"Map contains {non_zero} non-unknown cells")
            
            if non_zero == 0:
                self.get_logger().warn("Map appears to be empty (all cells are unknown)!")
    
    def debug_map_callback(self, msg):
        if not self.got_map_from_debug_topic:
            self.got_map_from_debug_topic = True
            self.get_logger().info('Received map data from debug_map topic!')
            self.get_logger().info(f"Debug map frame_id: {msg.header.frame_id}")
            self.get_logger().info(f"Debug map dimensions: {msg.info.width}x{msg.info.height} cells")
            self.get_logger().info(f"Debug map resolution: {msg.info.resolution} meters/cell")
            
            # Check if map has actual data
            non_zero = sum(1 for cell in msg.data if cell != -1)
            self.get_logger().info(f"Debug map contains {non_zero} non-unknown cells")
            
            if non_zero == 0:
                self.get_logger().warn("Debug map appears to be empty (all cells are unknown)!")
    
    def service_timer_callback(self):
        # Cancel this timer after first run
        self.service_timer.cancel()
        
        # Use threading to avoid blocking
        import threading
        thread = threading.Thread(target=self.check_service_with_timeout)
        thread.daemon = True
        thread.start()
    
    def check_service_with_timeout(self):
        if self.map_client.service_is_ready():
            self.get_logger().info('Map service is available. Requesting map...')
            
            try:
                # Create a future for the service call
                future = self.map_client.call_async(GetMap.Request())
                
                # Set a timeout for the service call
                timeout = self.service_timeout
                start_time = time.time()
                
                # Poll the future until timeout
                while time.time() - start_time < timeout:
                    if future.done():
                        self.get_logger().info('Map service responded!')
                        response = future.result()
                        if response:
                            self.got_map_from_service = True
                            self.get_logger().info(f"Map from service has dimensions: {response.map.info.width}x{response.map.info.height}")
                            non_zero = sum(1 for cell in response.map.data if cell != -1)
                            self.get_logger().info(f"Map from service contains {non_zero} non-unknown cells")
                        else:
                            self.get_logger().error("Map service responded but returned None")
                        return
                    time.sleep(0.1)
                
                self.get_logger().error(f"Map service request timed out after {timeout} seconds")
                self.get_logger().info("Possible issues:")
                self.get_logger().info("1. The map server is not in the 'active' lifecycle state")
                self.get_logger().info("2. The map server is not properly configured")
                self.get_logger().info("3. The map service has a different name than expected")
                self.get_logger().info("4. The map server is busy or overloaded")
                self.get_logger().info("\nTry running: ros2 lifecycle set /map_server configure")
                self.get_logger().info("Then: ros2 lifecycle set /map_server activate")
            
            except Exception as e:
                self.get_logger().error(f"Error calling map service: {str(e)}")
        else:
            self.get_logger().error("Map service is not available. Check if map_server is running.")
            self.get_logger().info("Try running: ros2 service list | grep map")
            self.get_logger().info("To manually get the map service name")

def main():
    parser = argparse.ArgumentParser(description='Check map configuration and data')
    parser.add_argument('--map', type=str, help='Path to map YAML file')
    parser.add_argument('--timeout', type=float, default=2.0, help='Service call timeout in seconds')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = MapChecker(args.map, args.timeout)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
