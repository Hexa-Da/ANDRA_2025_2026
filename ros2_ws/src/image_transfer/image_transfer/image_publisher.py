import rclpy
from rclpy.node import Node
import subprocess
import time
import socket
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import numpy as np
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'photo_topic', 10)
        self.bridge = CvBridge()

        # Paramètres configurables pour la caméra PTZ
        self.declare_parameter('capture_interval', 10.0)  # secondes
        self.declare_parameter('enable_ptz', True)
        
        # Paramètres d'ajustement d'image
        self.declare_parameter('brightness', 1.0)  # Multiplicateur de luminosité (1.0 = normal, >1.0 = plus clair)
        self.declare_parameter('contrast', 1.0)  # Multiplicateur de contraste (1.0 = normal, >1.0 = plus de contraste)
        self.declare_parameter('gamma', 1.0)  # Correction gamma (1.0 = normal, <1.0 = plus clair)
        self.declare_parameter('enable_image_adjustment', False)  # Activer/désactiver l'ajustement
        
        # Paramètres de sauvegarde
        self.declare_parameter('save_all_images', True)  # Sauvegarder toutes les images capturées
        self.declare_parameter('images_output_dir', 'ros2_ws/images_capturees')  # Dossier de sauvegarde

        # Mode de capture
        # False = Prend des photos tout seul (Timer)
        # True  = Attend qu'on lui dise "trigger" (Scan)
        self.declare_parameter('mode_manuel', False) 
        self.mode_manuel = self.get_parameter('mode_manuel').get_parameter_value().bool_value

        # Récupérer les paramètres
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
        self.rtsp_url = f"rtsp://admin:admin@192.168.5.163:554/live/av0"
        
        self.get_logger().info(f"Configuration PTZ : 192.168.5.163:554")
        self.get_logger().info(f"URL RTSP : rtsp://admin:admin@192.168.5.163:554/live/av0")
        self.get_logger().info(f"PTZ activée : {enable_ptz}, Intervalle de capture : {capture_interval}s")
        if self.enable_adjustment:
            self.get_logger().info(f"Ajustement d'image : Mode MANUEL - Luminosité={self.brightness}, Contraste={self.contrast}, Gamma={self.gamma}")

        self.set_auto_exposure()

        # Capture une image en manuel via un topic ou en automatique via un timer
        if enable_ptz:
            if self.mode_manuel:
                # CAS 1 : MODE SCAN (Manuel)
                self.get_logger().info("--- MODE MANUEL ACTIVÉ : En attente d'ordre sur /trigger_capture ---")
                self.trigger_sub = self.create_subscription(
                    Bool, 
                    'trigger_capture', 
                    self.trigger_callback, 
                    10
                )
                self.timer = None # On s'assure qu'il n'y a pas de timer
            else:
                # CAS 2 : MODE AUTO (Classique)
                self.get_logger().info(f"--- MODE AUTOMATIQUE : Capture toutes les {capture_interval}s ---")
                self.timer = self.create_timer(capture_interval, self.capture_and_publish)

    def trigger_callback(self, msg):
            """Appelée uniquement quand scan_manager envoie un ordre"""
            if msg.data:
                self.get_logger().info(">> SIGNAL TRIGGER REÇU : Capture en cours...")
                self.capture_and_publish()

    def set_auto_exposure(self):
            """Active le mode Full Auto via une connexion VISCA temporaire"""
            cam_ip = '192.168.5.163'
            cam_port = 1259  # Port VISCA Marshall
            
            try:
                # 1. Création d'une connexion temporaire
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as temp_sock:
                    temp_sock.settimeout(2.0)
                    temp_sock.connect((cam_ip, cam_port))
                    
                    # 2. Commande VISCA Full Auto: 81 01 04 39 00 FF
                    # 0x81 (Header), 0x01 0x04 0x39 0x00 (Cmd), 0xFF (Terminator)
                    command = bytes([0x81, 0x01, 0x04, 0x39, 0x00, 0xFF])
                    
                    temp_sock.sendall(command)
                    
                    self.get_logger().info('HARDWARE: Mode Auto-Exposure activé avec succès sur la caméra.')
                    
            except Exception as e:
                self.get_logger().error(f'Impossible d\'activer l\'auto-exposure matériel : {e}')

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
        Ajustement manuel avec valeurs fixes (brightness, contrast, gamma)
        """
        # Convertir en float pour éviter les problèmes de saturation
        img_float = img.astype(np.float32)
        
        # 1. Appliquer la luminosité (multiplication)
        img_adjusted = img_float * self.brightness
        
        # 2. Appliquer le contraste (centré sur 128)
        # Formule : output = (input - 128) * contrast + 128
        img_adjusted = (img_adjusted - 128.0) * self.contrast + 128.0
        
        # 3. Appliquer gamma si nécessaire
        if self.gamma != 1.0:
            # Normaliser
            img_norm = np.clip(img_adjusted / 255.0, 0.0, 1.0)
            # Appliquer gamma
            img_gamma = np.power(img_norm, 1.0 / self.gamma)
            # Reconvertir
            img_adjusted = img_gamma * 255.0
        
        # Saturation et conversion
        img_adjusted = np.clip(img_adjusted, 0, 255).astype(np.uint8)
        
        return img_adjusted

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
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            image_publisher.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # Ignorer l'erreur si rclpy est déjà shutdown
            pass

if __name__ == '__main__':
    main()
