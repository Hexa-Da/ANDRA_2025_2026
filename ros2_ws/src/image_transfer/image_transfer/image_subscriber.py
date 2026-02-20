import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from ultralytics import YOLO
from geometry_msgs.msg import Point
import cv2
import os
import time
import torch


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.position_pub = self.create_publisher(Point, 'position_detectee', 10)
        self.position = None
        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info("Abonne au topic /photo_topic")
        self.image_count = 0

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if torch.cuda.is_available():
            try:
                gpu_name = torch.cuda.get_device_name(0)
                self.get_logger().info(f"CUDA disponible: {gpu_name}")
                # Garder une marge memoire sur Jetson pour eviter les erreurs NvMap.
                torch.cuda.set_per_process_memory_fraction(0.5)
            except Exception as e:
                self.get_logger().warn(f"Probleme CUDA: {e}, fallback CPU")
                self.device = 'cpu'
        else:
            self.get_logger().warn("CUDA non disponible, fallback CPU")

        # Chercher d'abord TensorRT (.engine), puis PyTorch (.pt)
        # TensorRT est beaucoup plus rapide sur Jetson
        docker_engine = '/ros2_ws/models/best.engine'
        docker_pt = '/ros2_ws/models/best.pt'
        local_engine = '/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/models/best.engine'
        local_pt = '/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/models/best.pt'
        
        # Recherche du modele YOLO (TensorRT prioritaire)
        model_candidates = [
            (docker_engine, "TensorRT"), (docker_pt, "PyTorch"),
            (local_engine, "TensorRT"), (local_pt, "PyTorch")
        ]
        
        # Priorite: TensorRT (.engine) avant PyTorch (.pt)
        force_pytorch = os.environ.get('FORCE_PYTORCH', '0') == '1'
        if force_pytorch:
            self.get_logger().warn("FORCE_PYTORCH=1: TensorRT desactive")
        
        self.model_path = None
        self.use_tensorrt = False
        for path, model_type in model_candidates:
            if force_pytorch and model_type == "TensorRT":
                continue
            if os.path.exists(path) and os.access(path, os.R_OK):
                self.model_path = path
                self.use_tensorrt = (model_type == "TensorRT")
                self.get_logger().info(f"Modele {model_type}: {path}")
                break
        
        if self.model_path is None:
            raise FileNotFoundError("Modele YOLO non trouve (best.engine ou best.pt)")
        
        self.model = None
        self._model_loaded = False
        self.get_logger().info("Modele charge au premier callback (lazy loading)")
        # Reglages d'inference conservateurs pour tenir sur GPU Jetson.
        self.gpu_imgsz = 640
        # Option pour dessiner les boxes (desactivable pour performance)
        self.draw_boxes = True
        # Compteur d'erreurs consecutives pour fallback automatique
        self._consecutive_errors = 0
        self._max_errors_before_fallback = 5

        # Dans Docker, toujours ecrire dans le volume monte /ros2_ws
        # pour retrouver les images detectees cote hote.
        if os.path.isdir('/ros2_ws') or os.path.exists('/.dockerenv'):
            self.output_dir = '/ros2_ws/images_detectees'
        else:
            project_root = os.path.expanduser("~/Documents/ANDRA_2025-2026")
            self.output_dir = os.path.join(project_root, "ros2_ws", "images_detectees")
        
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Dossier detections: {self.output_dir}")

        self.get_logger().info('image_subscriber pret')
    
    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position

    def listener_callback(self, msg):
        """Recoit une image sur /photo_topic puis lance YOLO."""
        self.image_count += 1
        if self.image_count == 1:
            self.get_logger().info("Premiere image recue sur /photo_topic")
        elif self.image_count % 50 == 0:
            self.get_logger().info(f"{self.image_count} images recues")
        
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image: {e}")
            return

        detected, img_with_boxes = self.detect_tags(img)

        if detected:
            self.get_logger().info("Detection fissure")
            # Timestamp haute resolution pour eviter l'ecrasement (plusieurs detections/s)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            ms = int((time.time() % 1) * 1000)
            image_path = os.path.join(
                self.output_dir,
                f"detection_{timestamp}_{ms:03d}_{self.image_count:06d}.jpg"
            )
            cv2.imwrite(image_path, img_with_boxes)
            self.get_logger().info(f"Image sauvegardee: {image_path}")
            if self.position is not None:
                point_msg = Point()
                point_msg.x = self.position.x
                point_msg.y = self.position.y
                point_msg.z = self.position.z
                self.position_pub.publish(point_msg)
                self.get_logger().info("Position detectee publiee")

    def _load_model(self):
        """Charge YOLO au premier callback."""
        if self._model_loaded:
            return
        
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                try:
                    gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
                    gpu_allocated = torch.cuda.memory_allocated(0) / 1024**3
                    gpu_free = gpu_memory - gpu_allocated
                    self.get_logger().info(f"Memoire GPU libre: {gpu_free:.2f} / {gpu_memory:.2f} GB")
                    if gpu_free < 1.0:
                        self.get_logger().warn("Memoire GPU insuffisante (<1GB), fallback CPU")
                        self.device = 'cpu'
                except Exception as e:
                    self.get_logger().warn(f"Impossible de lire la memoire GPU: {e}, fallback CPU")
                    self.device = 'cpu'

            self.get_logger().info("Chargement modele YOLO...")
            self.model = YOLO(self.model_path)
            self._model_loaded = True
            self.get_logger().info(f"Modele charge. Device inference cible: {self.device}")
            time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f"Erreur chargement modele: {e}")
            raise
    
    def detect_tags(self, img):
        """Applique YOLO et retourne (detected, image_annotee)."""
        try:
            if not self._model_loaded:
                self._load_model()

            device_id = 0 if self.device == 'cuda' and torch.cuda.is_available() else 'cpu'
            try:
                if device_id == 0:
                    if self.use_tensorrt:
                        # TensorRT: deja optimise pour GPU, pas besoin de half/imgsz
                        results = self.model(img, device=0, verbose=False)
                    else:
                        # PyTorch: FP16 + image plus petite pour reduire memoire CUDA
                        results = self.model(
                            img,
                            device=0,
                            imgsz=self.gpu_imgsz,
                            half=True,
                            verbose=False
                        )
                else:
                    results = self.model(img, device='cpu', imgsz=self.gpu_imgsz, verbose=False)
                
                # Reset compteur si inference reussie
                self._consecutive_errors = 0
                
            except Exception as e:
                self._consecutive_errors += 1
                
                # Si trop d'erreurs consecutives avec TensorRT, basculer sur CPU
                if self.use_tensorrt and self._consecutive_errors >= self._max_errors_before_fallback:
                    self.get_logger().error(
                        f"{self._consecutive_errors} erreurs TensorRT consecutives - "
                        "regenerer .engine avec: model.export(format='engine', device=0)"
                    )
                    self.device = 'cpu'
                    self._consecutive_errors = 0
                
                # Fallback robuste en cas de crash/erreur CUDA a l'inference.
                if device_id != 'cpu':
                    self.get_logger().warn(f"Erreur inference GPU ({type(e).__name__}: {e}), fallback CPU")
                    self.device = 'cpu'
                    try:
                        torch.cuda.empty_cache()
                    except Exception:
                        pass
                    results = self.model(img, device='cpu', imgsz=self.gpu_imgsz, verbose=False)
                else:
                    raise

            detected = len(results[0].boxes) > 0
            
            # plot() dessine les bounding boxes sur l'image (operation CPU)
            # Si draw_boxes=False, on retourne l'image originale (plus rapide)
            if detected:
                if self.draw_boxes:
                    img_with_boxes = results[0].plot()  # Dessine les boxes (CPU)
                else:
                    img_with_boxes = img  # Pas de dessin = plus rapide
            else:
                img_with_boxes = img
                
            return detected, img_with_boxes
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"Erreur detect_tags: {type(e).__name__}: {e}")
            self.get_logger().debug(f"Traceback: {traceback.format_exc()}")
            return False, img

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            image_subscriber.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # Ignorer l'erreur si rclpy est déjà shutdown
            pass

if __name__ == '__main__':
    main()

