import gc
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from ultralytics import YOLO
from geometry_msgs.msg import Point
from typing import Optional, Tuple
import cv2
import numpy as np
import os
import time
import torch


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Callback groups : isole l'inférence YOLO (lente) du callback odométrie
        # et des futures abonnements légers, pour ne pas bloquer le spin global.
        self._inference_cb_group = MutuallyExclusiveCallbackGroup()
        self._sensor_cb_group = MutuallyExclusiveCallbackGroup()

        self.position_pub = self.create_publisher(Point, 'position_detectee', 10)
        self.position = None
        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.listener_callback,
            10,
            callback_group=self._inference_cb_group,
        )
        self.get_logger().info("Abonne au topic /photo_topic")
        self.image_count = 0

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10,
            callback_group=self._sensor_cb_group,
        )

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._gpu_available = torch.cuda.is_available()
        # Configurable via env: GPU_RETRY_INTERVAL (default 50 images)
        self._gpu_retry_interval = int(os.environ.get('GPU_RETRY_INTERVAL', '50'))
        self._images_since_gpu_failure = 0
        
        if self._gpu_available:
            try:
                gpu_name = torch.cuda.get_device_name(0)
                self.get_logger().info(f"CUDA disponible: {gpu_name}")
                # Allocation dynamique - pas de reservation fixe pour partager avec ZED/SLAM
                torch.cuda.empty_cache()
            except Exception as e:
                self.get_logger().warn(f"Probleme init CUDA: {e}")
                # Ne pas abandonner le GPU, reessayer plus tard
        else:
            self.get_logger().warn("CUDA non disponible, fallback CPU")

        # Chemins des modeles (Docker et local)
        docker_engine = '/ros2_ws/models/best.engine'
        docker_pt = '/ros2_ws/models/best.pt'
        local_engine = '/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/models/best.engine'
        local_pt = '/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/models/best.pt'
        
        # Trouver les modeles disponibles
        self.engine_path = None
        self.pytorch_path = None
        for engine in [docker_engine, local_engine]:
            if os.path.exists(engine) and os.access(engine, os.R_OK):
                self.engine_path = engine
                break
        for pt in [docker_pt, local_pt]:
            if os.path.exists(pt) and os.access(pt, os.R_OK):
                self.pytorch_path = pt
                break
        
        # Priorite: TensorRT si GPU disponible, sinon PyTorch
        force_pytorch = os.environ.get('FORCE_PYTORCH', '0') == '1'
        if force_pytorch:
            self.get_logger().warn("FORCE_PYTORCH=1: TensorRT desactive")
        
        self.use_tensorrt = False
        if self.engine_path and self._gpu_available and not force_pytorch:
            self.model_path = self.engine_path
            self.use_tensorrt = True
            self.get_logger().info(f"Modele TensorRT: {self.engine_path}")
        elif self.pytorch_path:
            self.model_path = self.pytorch_path
            self.get_logger().info(f"Modele PyTorch: {self.pytorch_path}")
        else:
            raise FileNotFoundError("Modele YOLO non trouve (best.engine ou best.pt)")
        
        self.model: Optional[YOLO] = None
        self._model_loaded = False
        self._tensorrt_disabled = False
        # Jetson + ZED/AMCL : TensorRT OK vers 3.3 Go CUDA, echoue vers 2.4 Go (NvMap).
        self._min_gpu_free_gb = float(os.environ.get('MIN_GPU_FREE_GB', '3.0'))
        self.get_logger().info(
            f"Modele charge au premier callback (lazy loading, seuil TensorRT: "
            f"{self._min_gpu_free_gb} GB CUDA)"
        )
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

    def _get_gpu_free_gb(self) -> Optional[Tuple[float, float]]:
        """
        Retourne (libre_GB, total_GB) via cudaMemGetInfo (mem_get_info).
        Precondition: CUDA disponible.
        Invariant: ne compte pas seulement les allocations PyTorch (ZED/SLAM partagent le pool).
        """
        if not self._gpu_available:
            return None
        try:
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            free_bytes, total_bytes = torch.cuda.mem_get_info(0)
            free_gb = free_bytes / (1024 ** 3)
            total_gb = total_bytes / (1024 ** 3)
            return free_gb, total_gb
        except Exception as e:
            self.get_logger().warn(f"Impossible de verifier GPU: {e}")
            return None

    def _check_gpu_memory(self, min_gb: float = 1.0) -> bool:
        """Verifie si assez de memoire GPU CUDA est disponible (tous processus confondus)."""
        stats = self._get_gpu_free_gb()
        if stats is None:
            return False
        free_gb, total_gb = stats
        self.get_logger().info(f"GPU CUDA: {free_gb:.2f}/{total_gb:.2f} GB libre")
        return free_gb >= min_gb

    def _unload_model(self) -> None:
        """
        Libere le modele YOLO/TensorRT avant un changement de backend.
        Precondition: appele avant de recharger un autre modele (evite segfault NvMap/TRT).
        """
        self.model = None
        self._model_loaded = False
        gc.collect()
        if self._gpu_available:
            try:
                torch.cuda.synchronize()
                torch.cuda.empty_cache()
            except Exception:
                pass

    def _warmup_tensorrt(self) -> bool:
        """
        Premiere inference test : Ultralytics deserialise l'engine TRT ici, pas au YOLO().
        Precondition: self.model charge avec best.engine.
        """
        if self.model is None:
            return False
        try:
            dummy: np.ndarray = np.zeros(
                (self.gpu_imgsz, self.gpu_imgsz, 3), dtype=np.uint8
            )
            self.model(dummy, device=0, verbose=False)
            return True
        except Exception as e:
            self.get_logger().warn(f"Warmup TensorRT echoue: {type(e).__name__}: {e}")
            return False

    def _switch_to_pytorch_cpu(self, reason: str) -> None:
        """Bascule vers best.pt sur CPU apres echec ou manque de memoire GPU."""
        if not self.pytorch_path:
            raise RuntimeError("Pas de modele PyTorch (best.pt) pour le fallback CPU")
        self.get_logger().warn(f"{reason} -> fallback PyTorch CPU")
        self._unload_model()
        self._tensorrt_disabled = True
        self.model_path = self.pytorch_path
        self.use_tensorrt = False
        self.device = 'cpu'

    def _load_pytorch_cpu_model(self) -> None:
        """
        Charge best.pt sur CPU.
        Precondition: self.model_path pointe vers best.pt, pas de modele actif.
        Postcondition: self._model_loaded True seulement si YOLO() reussit.
        """
        try:
            self.model = YOLO(self.model_path, task='segment')
        except Exception:
            self._unload_model()
            raise
        self._model_loaded = True

    def _load_model(self) -> None:
        """Charge YOLO au premier callback."""
        if self._model_loaded:
            return

        try:
            if self.use_tensorrt and self._tensorrt_disabled:
                self._switch_to_pytorch_cpu("TensorRT desactive pour cette session")

            if self.use_tensorrt:
                if not self._check_gpu_memory(min_gb=self._min_gpu_free_gb):
                    self._switch_to_pytorch_cpu(
                        f"Memoire GPU < {self._min_gpu_free_gb} GB (reserve ZED/SLAM)"
                    )

            self.get_logger().info(f"Chargement {'TensorRT' if self.use_tensorrt else 'PyTorch'}...")
            self.model = YOLO(self.model_path, task='segment')

            if self.use_tensorrt:
                if not self._warmup_tensorrt():
                    if not self.pytorch_path:
                        raise RuntimeError("Warmup TensorRT echoue, pas de best.pt")
                    self._switch_to_pytorch_cpu("Warmup TensorRT (NvMap/CUDA)")
                    self._load_pytorch_cpu_model()
                    self.get_logger().info("Fallback PyTorch CPU reussi (warmup)")
                else:
                    self.get_logger().info("TensorRT valide (warmup GPU OK)")
            else:
                self.get_logger().info(f"PyTorch charge (device: {self.device})")

            self._model_loaded = True

        except Exception as e:
            self.get_logger().error(f"Erreur chargement modele: {e}")
            if self.use_tensorrt and self.pytorch_path:
                self._switch_to_pytorch_cpu("Echec chargement TensorRT")
                try:
                    self._load_pytorch_cpu_model()
                    self.get_logger().info("Fallback PyTorch CPU reussi")
                    return
                except Exception as e2:
                    self.get_logger().error(f"Echec fallback PyTorch: {e2}")
            raise
    
    def detect_tags(self, img):
        """Applique YOLO et retourne (detected, image_annotee)."""
        try:
            if not self._model_loaded:
                self._load_model()

            # Retry GPU periodiquement apres un fallback CPU (PyTorch seulement)
            if self.device == 'cpu' and self._gpu_available and not self.use_tensorrt:
                self._images_since_gpu_failure += 1
                if self._images_since_gpu_failure >= self._gpu_retry_interval:
                    self._images_since_gpu_failure = 0
                    if self._check_gpu_memory(min_gb=0.5):
                        self.device = 'cuda'
                        self.get_logger().info("Retour sur GPU (PyTorch)")

            device_id = 0 if self.device == 'cuda' and self._gpu_available else 'cpu'
            
            try:
                # Nettoyer le cache GPU avant inference pour liberer la memoire
                if device_id == 0:
                    torch.cuda.empty_cache()
                
                if device_id == 0:
                    if self.use_tensorrt:
                        results = self.model(img, device=0, verbose=False)
                    else:
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
                self._images_since_gpu_failure = 0
                
            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().warn(f"Erreur inference ({type(e).__name__}): {e}")
                
                # TensorRT ne peut PAS tourner sur CPU - basculer sur PyTorch
                if self.use_tensorrt and self.pytorch_path:
                    self._switch_to_pytorch_cpu("Erreur TensorRT")
                    try:
                        self._load_pytorch_cpu_model()
                        self.get_logger().info(
                            "Modele PyTorch charge, retry inference..."
                        )
                        results = self.model(
                            img,
                            device='cpu',
                            imgsz=self.gpu_imgsz,
                            verbose=False,
                        )
                    except Exception:
                        self._unload_model()
                        raise
                # PyTorch: fallback GPU -> CPU
                elif device_id != 'cpu' and not self.use_tensorrt:
                    self.get_logger().warn("Fallback PyTorch CPU")
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
            if detected and self.draw_boxes:
                try:
                    img_with_boxes = results[0].plot()
                except KeyError:
                    # TensorRT: noms de classes non disponibles, dessiner manuellement
                    img_with_boxes = img.copy()
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cv2.rectangle(img_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        conf = float(box.conf[0])
                        cv2.putText(img_with_boxes, f"{conf:.2f}", (x1, y1-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
    # MultiThreadedExecutor : permet à odom_callback de tourner pendant que
    # listener_callback fait l'inférence YOLO (callback groups séparés).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(image_subscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            image_subscriber._unload_model()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            image_subscriber.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

