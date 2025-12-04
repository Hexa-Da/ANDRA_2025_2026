#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from PIL import Image
import matplotlib.pyplot as plt
import yaml
import os
from datetime import datetime


def tracer_point(point):
    yaml_path = "andra.yaml"
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    pgm_path = data['image']
    resolution = data['resolution']
    origin = data['origin'][:2]

    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    img = Image.open(pgm_path)
    width, height = img.size

    plt.figure()
    plt.imshow(img, cmap='gray', origin='upper')

    pixel_x = (point[0] - origin[0]) / resolution
    pixel_y = (height - 1) - ((point[1] - origin[1]) / resolution)

    plt.scatter(pixel_x, pixel_y, c='red', s=50, label=f'{point}')
    plt.axis('off')
    plt.title("Carte + Point détecté")

    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    output_filename = f"map_with_point_{timestamp}.png"

    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0.1)
    print(f"[Carte] Image enregistrée sous : {output_filename}")
    plt.close()


class MapPointPlotter(Node):
    def __init__(self):
        super().__init__('map_point_plotter')
        self.subscription = self.create_subscription(
            Point,
            'position_detectee',
            self.listener_callback,
            10)
        self.get_logger().info('MapPointPlotter prêt et abonné à /position_detectee')

    def listener_callback(self, msg):
        self.get_logger().info(f'Point reçu : ({msg.x}, {msg.y})')
        tracer_point((msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    node = MapPointPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # Ignorer l'erreur si rclpy est déjà shutdown
            pass


if __name__ == '__main__':
    main()

