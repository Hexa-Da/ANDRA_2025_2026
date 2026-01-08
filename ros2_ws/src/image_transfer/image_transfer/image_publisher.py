import rclpy
from rclpy.node import Node
import subprocess
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class Noeud3(Node):
    def __init__(self):
        super().__init__('noeud_3')
        self.publisher = self.create_publisher(Image, 'photo_topic', 10)
        self.bridge = CvBridge()

        # Paramètres configurables pour la caméra PTZ
        self.declare_parameter('ptz_ip', '192.168.5.163')
        self.declare_parameter('ptz_port', 554)
        self.declare_parameter('ptz_username', 'admin')
        self.declare_parameter('ptz_password', 'admin')
        self.declare_parameter('ptz_stream_path', 'live/av0')
        self.declare_parameter('capture_interval', 10.0)  # secondes
        self.declare_parameter('enable_ptz', True)
        
        # Paramètres d'ajustement d'image
        self.declare_parameter('brightness', 1.5)  # Multiplicateur de luminosité (1.0 = normal, >1.0 = plus clair)
        self.declare_parameter('contrast', 1.25)  # Multiplicateur de contraste (1.0 = normal, >1.0 = plus de contraste)
        self.declare_parameter('gamma', 1.0)  # Correction gamma (1.0 = normal, <1.0 = plus clair)
        self.declare_parameter('enable_image_adjustment', True)  # Activer/désactiver l'ajustement
        
        # Paramètres de sauvegarde
        self.declare_parameter('save_all_images', True)  # Sauvegarder toutes les images capturées
        self.declare_parameter('images_output_dir', 'ros2_ws/images_capturees')  # Dossier de sauvegarde

        # Récupérer les paramètres
        ptz_ip = self.get_parameter('ptz_ip').get_parameter_value().string_value
        ptz_port = self.get_parameter('ptz_port').get_parameter_value().integer_value
        ptz_username = self.get_parameter('ptz_username').get_parameter_value().string_value
        ptz_password = self.get_parameter('ptz_password').get_parameter_value().string_value
        ptz_stream_path = self.get_parameter('ptz_stream_path').get_parameter_value().string_value
        capture_interval = self.get_parameter('capture_interval').get_parameter_value().double_value
        enable_ptz = self.get_parameter('enable_ptz').get_parameter_value().bool_value
        
        # Paramètres d'ajustement d'image
        self.brightness = self.get_parameter('brightness').get_parameter_value().double_value
        self.contrast = self.get_parameter('contrast').get_parameter_value().double_value
        self.gamma = self.get_parameter('gamma').get_parameter_value().double_value
        self.enable_adjustment = self.get_parameter('enable_image_adjustment').get_parameter_value().bool_value
        
        # Paramètres de sauvegarde
        self.save_all_images = self.get_parameter('save_all_images').get_parameter_value().bool_value
        images_output_dir = self.get_parameter('images_output_dir').get_parameter_value().string_value
        
        # Créer le dossier de sauvegarde si nécessaire
        if self.save_all_images:
            # Convertir le chemin relatif en absolu si nécessaire
            if not os.path.isabs(images_output_dir):
                project_root = os.path.expanduser("~/Documents/ANDRA_2025-2026")
                self.images_output_dir = os.path.join(project_root, images_output_dir)
            else:
                self.images_output_dir = images_output_dir
            
            if not os.path.exists(self.images_output_dir):
                os.makedirs(self.images_output_dir)
                self.get_logger().info(f'Dossier créé : {self.images_output_dir}')
            
        self.image_count = 0

        # Construire l'URL RTSP
        self.rtsp_url = f"rtsp://{ptz_username}:{ptz_password}@{ptz_ip}:{ptz_port}/{ptz_stream_path}"
        
        self.get_logger().info(f"Configuration PTZ : {ptz_ip}:{ptz_port}")
        self.get_logger().info(f"URL RTSP : rtsp://{ptz_username}:***@{ptz_ip}:{ptz_port}/{ptz_stream_path}")
        self.get_logger().info(f"PTZ activée : {enable_ptz}, Intervalle de capture : {capture_interval}s")
        if self.enable_adjustment:
            self.get_logger().info(f"Ajustement d'image : Luminosité={self.brightness}, Contraste={self.contrast}, Gamma={self.gamma}")

        # Capture une image toutes les X secondes (configurable)
        if enable_ptz:
            self.timer = self.create_timer(capture_interval, self.capture_and_publish)

    def capture_and_publish(self):
        self.get_logger().info("Capture d'une image depuis la caméra PTZ...")

        # Exécute FFmpeg et récupère l'image directement en mémoire
        ffmpeg_command = [
            "ffmpeg", "-i", self.rtsp_url,
            "-vframes", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"
        ]

        # Récupérer les erreurs de la commande FFmpeg
        process = subprocess.run(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)    
        
        if process.returncode == 0:
            # Convertir le flux binaire en image OpenCV
            np_arr = np.frombuffer(process.stdout, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                # Appliquer les ajustements d'image si activés
                if self.enable_adjustment:
                    img = self.adjust_image(img)
                
                # Sauvegarder l'image si activé
                if self.save_all_images:
                    self.save_image(img)
                
                # Convertir l'image en message ROS 2
                ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.publisher.publish(ros_img)
            else:
                self.get_logger().error("Erreur lors du décodage de l'image capturée.")
        else:
            error_msg = process.stderr.decode('utf-8') if process.stderr else "Erreur inconnue"
            self.get_logger().error(f"Erreur lors de la capture avec FFmpeg : {error_msg}")
            self.get_logger().error(f"Vérifiez la connexion réseau et l'accessibilité de la caméra PTZ")

    def adjust_image(self, img):
        """
        Ajuste la luminosité, le contraste et le gamma d'une image
        """
        img = cv2.convertScaleAbs(img, alpha=self.contrast, beta=self.brightness)
        if self.gamma != 1.0:
            img = cv2.convertScaleAbs(img, alpha=self.gamma, beta=0)
        return img  

    def save_image(self, img):
        """
        Sauvegarde l'image capturée avec un timestamp
        """
        try:
            self.image_count += 1
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            image_filename = f"image_{self.image_count:04d}_{timestamp}.jpg"
            image_path = os.path.join(self.images_output_dir, image_filename)
            cv2.imwrite(image_path, img) 
            self.get_logger().info(f'Image sauvegardée : {image_filename} ({img.shape[1]}x{img.shape[0]})')
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde : {e}')

def main(args=None):
    rclpy.init(args=args)
    noeud_3 = Noeud3()

    try:
        rclpy.spin(noeud_3)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            noeud_3.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # Ignorer l'erreur si rclpy est déjà shutdown
            pass

if __name__ == '__main__':
    main()
