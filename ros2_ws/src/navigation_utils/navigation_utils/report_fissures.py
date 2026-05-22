#!/usr/bin/env python3

import os
from datetime import datetime
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point
from PIL import Image
from rclpy.node import Node


def default_map_yaml_path() -> str:
    """
    Carte par défaut dans ros_launcher (lookup runtime, sans exec_depend pour éviter le cycle colcon).
    Postcondition : chemin absolu ou chaîne vide si ros_launcher absent.
    """
    try:
        share_dir: str = get_package_share_directory('ros_launcher')
    except PackageNotFoundError:
        return ''
    return os.path.join(share_dir, 'map_results', 'ma_carte_2.yaml')


def resolve_map_yaml_path(yaml_path_param: str) -> str:
    """
    Précondition : yaml_path_param est une chaîne (vide = ma_carte_2.yaml via ros_launcher si installé).
    Postcondition : chemin absolu vers un fichier .yaml de carte, ou chaîne vide si introuvable.
    """
    if not yaml_path_param.strip():
        return default_map_yaml_path()
    if os.path.isabs(yaml_path_param):
        return yaml_path_param
    return os.path.abspath(yaml_path_param)


def resolve_output_dir(output_dir_param: str) -> str:
    """Précondition : output_dir_param non vide ou vide (défaut ros2_ws/map_detections)."""
    if not output_dir_param.strip():
        return os.path.abspath(os.path.join('ros2_ws', 'map_detections'))
    if os.path.isabs(output_dir_param):
        return output_dir_param
    return os.path.abspath(output_dir_param)


def tracer_point(
    point: Tuple[float, float],
    yaml_path: str,
    output_dir: str,
) -> Optional[str]:
    """
    Préconditions : yaml_path existe ; point = (x, y) dans le repère carte (m).
    Postcondition : retourne le chemin PNG créé, ou None si échec.
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        data: dict = yaml.safe_load(f)

    pgm_path: str = str(data['image'])
    resolution: float = float(data['resolution'])
    origin: list = data['origin'][:2]

    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    if not os.path.exists(pgm_path):
        print(f'ERREUR: Fichier PGM non trouvé : {pgm_path}')
        return None

    img = Image.open(pgm_path)
    width, height = img.size

    plt.figure()
    plt.imshow(img, cmap='gray', origin='upper')

    pixel_x: float = (point[0] - float(origin[0])) / resolution
    pixel_y: float = (height - 1) - ((point[1] - float(origin[1])) / resolution)

    plt.scatter(pixel_x, pixel_y, c='red', s=50, label=f'{point}')
    plt.axis('off')
    plt.title('Carte + Point détecté')

    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    os.makedirs(output_dir, exist_ok=True)
    timestamp: str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    output_filename: str = os.path.join(output_dir, f'map_with_point_{timestamp}.png')

    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0.1)
    plt.close()
    return output_filename


class MapPointPlotter(Node):
    def __init__(self) -> None:
        super().__init__('report_fissures')

        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('output_dir', '')

        yaml_path_param: str = (
            self.get_parameter('map_yaml_path').get_parameter_value().string_value
        )
        output_dir_param: str = (
            self.get_parameter('output_dir').get_parameter_value().string_value
        )

        self.yaml_path: str = resolve_map_yaml_path(yaml_path_param)
        self.output_dir: str = resolve_output_dir(output_dir_param)

        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f'Fichier YAML non trouvé : {self.yaml_path}')
            self.get_logger().error(
                'Fournir map_yaml_path ou installer/sourcer ros_launcher (ma_carte_2.yaml par défaut)'
            )
        else:
            self.get_logger().info(f'Carte : {self.yaml_path}')
            self.get_logger().info(f'Sortie PNG : {self.output_dir}')

        self.subscription = self.create_subscription(
            Point,
            'position_detectee',
            self.listener_callback,
            10,
        )
        self.get_logger().info('Abonné à /position_detectee')

    def listener_callback(self, msg: Point) -> None:
        self.get_logger().info(f'Détection à ({msg.x:.3f}, {msg.y:.3f})')
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f'Carte introuvable : {self.yaml_path}')
            return
        saved: Optional[str] = tracer_point(
            (float(msg.x), float(msg.y)),
            self.yaml_path,
            self.output_dir,
        )
        if saved:
            self.get_logger().info(f'Carte enregistrée : {saved}')


def main(args: Optional[list] = None) -> None:
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
            pass


if __name__ == '__main__':
    main()
