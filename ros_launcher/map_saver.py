#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import sys
import os

class MapSaverNode(Node):
    def __init__(self, filename):
        super().__init__('custom_map_saver')
        self.filename = filename
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.map_received = False
        self.get_logger().info('Waiting for map...')

    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info('Map received, saving...')
            
            # Save the metadata in YAML format
            with open(f'{self.filename}.yaml', 'w') as yaml_file:
                resolution = msg.info.resolution
                origin_x = msg.info.origin.position.x
                origin_y = msg.info.origin.position.y
                
                yaml_file.write(f"image: {self.filename}.pgm\n")
                yaml_file.write(f"resolution: {resolution}\n")
                yaml_file.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
                yaml_file.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            
            # Save the map as PGM
            with open(f'{self.filename}.pgm', 'wb') as pgm_file:
                width = msg.info.width
                height = msg.info.height
                
                # Write PGM header
                pgm_file.write(b"P5\n")
                pgm_file.write(b"# CREATOR: ROS2 Map Saver\n")
                pgm_file.write(f"{width} {height}\n".encode())
                pgm_file.write(b"255\n")
                
                # Write map data
                for i in range(height * width):
                    value = msg.data[i]
                    if value == -1:  # Unknown
                        pgm_file.write(bytes([205]))
                    elif value < 50:  # Free
                        pgm_file.write(bytes([254]))
                    else:  # Occupied
                        pgm_file.write(bytes([0]))
            
            self.get_logger().info(f'Map saved to {self.filename}.pgm and {self.filename}.yaml')
            self.map_received = True
            rclpy.shutdown()

def main():
    if len(sys.argv) < 2:
        print("Usage: ./custom_map_saver.py <filename>")
        return
    
    filename = sys.argv[1]
    
    rclpy.init()
    node = MapSaverNode(filename)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
