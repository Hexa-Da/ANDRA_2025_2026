#!/usr/bin/env python3

import os
from datetime import datetime
from typing import Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point, PointStamped
from PIL import Image, ImageDraw
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401 — enregistre PointStamped pour tf2
from tf2_ros import Buffer, TransformException, TransformListener


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

    with Image.open(pgm_path) as pgm_img:
        img: Image.Image = pgm_img.convert('RGB')
    width, height = img.size

    pixel_x: int = round((point[0] - float(origin[0])) / resolution)
    pixel_y: int = round((height - 1) - ((point[1] - float(origin[1])) / resolution))

    # Marqueur lisible, taille proportionnelle à la carte (export 1:1 pixels PGM).
    marker_radius: int = max(6, min(width, height) // 120)
    outline_w: int = max(2, marker_radius // 5)
    x0: int = pixel_x - marker_radius
    y0: int = pixel_y - marker_radius
    x1: int = pixel_x + marker_radius
    y1: int = pixel_y + marker_radius

    draw = ImageDraw.Draw(img)
    draw.ellipse(
        (x0, y0, x1, y1),
        fill='#e53935',
        outline='#ffffff',
        width=outline_w,
    )

    os.makedirs(output_dir, exist_ok=True)
    timestamp: str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    output_filename: str = os.path.join(output_dir, f'map_with_point_{timestamp}.png')

    img.save(output_filename, format='PNG', compress_level=1)
    return output_filename


class MapPointPlotter(Node):
    def __init__(self) -> None:
        super().__init__('report_fissures')

        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout_sec', 0.5)

        yaml_path_param: str = (
            self.get_parameter('map_yaml_path').get_parameter_value().string_value
        )
        output_dir_param: str = (
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.odom_frame: str = (
            self.get_parameter('odom_frame').get_parameter_value().string_value
        )
        self.map_frame: str = (
            self.get_parameter('map_frame').get_parameter_value().string_value
        )
        self.tf_timeout: Duration = Duration(
            seconds=float(
                self.get_parameter('tf_timeout_sec').get_parameter_value().double_value
            )
        )

        self.tf_buffer: Buffer = Buffer()
        self.tf_listener: TransformListener = TransformListener(self.tf_buffer, self)

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
        self.get_logger().info(
            f'TF : {self.odom_frame} → {self.map_frame} (aligné 2D Pose Estimate / AMCL)'
        )

    def _point_odom_to_map(self, msg: Point) -> Optional[Tuple[float, float]]:
        """
        Précondition : /position_detectee est en repère odom (EKF world_frame=odom).
        Postcondition : (x, y) en repère map, ou None si TF map←odom indisponible.
        """
        stamped_odom: PointStamped = PointStamped()
        stamped_odom.header.frame_id = self.odom_frame
        stamped_odom.header.stamp = Time(seconds=0).to_msg()
        stamped_odom.point = msg
        try:
            stamped_map: PointStamped = self.tf_buffer.transform(
                stamped_odom,
                self.map_frame,
                timeout=self.tf_timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self.odom_frame}→{self.map_frame} indisponible : {exc}. '
                'Lancer AMCL/SLAM et faire 2D Pose Estimate si besoin.'
            )
            return None
        return (float(stamped_map.point.x), float(stamped_map.point.y))

    def listener_callback(self, msg: Point) -> None:
        self.get_logger().info(
            f'Détection odom ({self.odom_frame}) : ({msg.x:.3f}, {msg.y:.3f})'
        )
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f'Carte introuvable : {self.yaml_path}')
            return
        map_xy: Optional[Tuple[float, float]] = self._point_odom_to_map(msg)
        if map_xy is None:
            return
        self.get_logger().info(
            f'Détection map ({self.map_frame}) : ({map_xy[0]:.3f}, {map_xy[1]:.3f})'
        )
        saved: Optional[str] = tracer_point(
            map_xy,
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
