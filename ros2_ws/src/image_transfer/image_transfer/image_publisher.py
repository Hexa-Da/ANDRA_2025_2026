import rclpy
from rclpy.node import Node
import subprocess
import time
import socket
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
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
        self.declare_parameter('enable_capture_auto', False)  # Capture automatique périodique

        # Paramètres réseau / identifiants PTZ (externalisés pour éviter le hardcoding)
        self.declare_parameter('ptz_host', '192.168.5.163')
        self.declare_parameter('ptz_rtsp_port', 554)
        self.declare_parameter('ptz_rtsp_path', '/live/av0')
        self.declare_parameter('ptz_visca_port', 1259)
        self.declare_parameter('ptz_user', 'admin')
        self.declare_parameter('ptz_password', 'admin')

        # Paramètres d'ajustement d'image
        self.declare_parameter('brightness', 1.0)  # Multiplicateur de luminosité (1.0 = normal, >1.0 = plus clair)
        self.declare_parameter('contrast', 1.0)  # Multiplicateur de contraste (1.0 = normal, >1.0 = plus de contraste)
        self.declare_parameter('gamma', 1.0)  # Correction gamma (1.0 = normal, <1.0 = plus clair)
        self.declare_parameter('enable_image_adjustment', False)  # Activer/désactiver l'ajustement
        
        # Paramètres de sauvegarde
        self.declare_parameter('save_all_images', True)  # Sauvegarder toutes les images capturées
        self.declare_parameter('images_output_dir', 'ros2_ws/images_capturees')  # Dossier de sauvegarde

        # Récupérer les paramètres
        capture_interval = self.get_parameter('capture_interval').get_parameter_value().double_value
        enable_ptz = self.get_parameter('enable_ptz').get_parameter_value().bool_value
        enable_capture_auto = self.get_parameter('enable_capture_auto').get_parameter_value().bool_value

        self.ptz_host = self.get_parameter('ptz_host').get_parameter_value().string_value
        self.ptz_rtsp_port = self.get_parameter('ptz_rtsp_port').get_parameter_value().integer_value
        self.ptz_rtsp_path = self.get_parameter('ptz_rtsp_path').get_parameter_value().string_value
        self.ptz_visca_port = self.get_parameter('ptz_visca_port').get_parameter_value().integer_value
        ptz_user = self.get_parameter('ptz_user').get_parameter_value().string_value
        ptz_password = self.get_parameter('ptz_password').get_parameter_value().string_value
        
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

        # Construire l'URL RTSP à partir des paramètres ROS
        rtsp_path = self.ptz_rtsp_path if self.ptz_rtsp_path.startswith('/') else f"/{self.ptz_rtsp_path}"
        self.rtsp_url = f"rtsp://{ptz_user}:{ptz_password}@{self.ptz_host}:{self.ptz_rtsp_port}{rtsp_path}"
        # Variante masquée des credentials pour les logs
        rtsp_url_redacted = f"rtsp://***:***@{self.ptz_host}:{self.ptz_rtsp_port}{rtsp_path}"

        self.get_logger().info(f"Configuration PTZ : {self.ptz_host}:{self.ptz_rtsp_port}")
        self.get_logger().info(f"URL RTSP : {rtsp_url_redacted}")
        self.get_logger().info(f"PTZ activée : {enable_ptz}, Intervalle de capture : {capture_interval}s")
        self.get_logger().info(f"Capture automatique : {enable_capture_auto}")
        if self.enable_adjustment:
            self.get_logger().info(f"Ajustement d'image : Mode MANUEL - Luminosité={self.brightness}, Contraste={self.contrast}, Gamma={self.gamma}")

        self.set_auto_exposure()

        # Subscriber pour déclencher des captures à la demande
        if enable_ptz:
            self.trigger_sub = self.create_subscription(
                String,
                '/trigger_capture',
                self.trigger_capture_callback,
                10
            )
            self.get_logger().info("Topic de déclenchement activé : /trigger_capture")
        
        # Capture automatique périodique (si activée)
        if enable_ptz and enable_capture_auto:
            self.timer = self.create_timer(capture_interval, self.capture_and_publish)
            self.get_logger().info(f"Capture automatique activée avec intervalle de {capture_interval}s")
        elif enable_ptz and not enable_capture_auto:
            self.get_logger().info("Capture automatique désactivée. Utilisez le topic /trigger_capture pour déclencher des captures.")

    def set_auto_exposure(self):
            """Active le mode Full Auto via une connexion VISCA temporaire"""
            cam_ip = self.ptz_host
            cam_port = self.ptz_visca_port  # Port VISCA Marshall

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

    def trigger_capture_callback(self, msg):
        """Callback pour déclencher une capture à la demande (msg.data = label optionnel)"""
        label = msg.data.strip() if msg.data else ""
        self.get_logger().info("Déclenchement de capture à la demande" + (f" ({label})" if label else "") + "...")
        self.capture_and_publish(label)

    def capture_and_publish(self, label=""):
        self.get_logger().info("Capture d'une image depuis la caméra PTZ...")

        # Exécute FFmpeg et récupère l'image directement en mémoire
        ffmpeg_command = [
            "ffmpeg", "-y", "-rtsp_transport", "tcp", "-i", self.rtsp_url,
            "-vframes", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"
        ]

        try:
            process = subprocess.run(
                ffmpeg_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"FFmpeg timeout (15s). Vérifiez : ping {self.ptz_host}, RTSP URL.")
            return
        except FileNotFoundError:
            self.get_logger().error("FFmpeg non trouvé. Installez : sudo apt install ffmpeg")
            return

        if process.returncode == 0:
            np_arr = np.frombuffer(process.stdout, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                if self.enable_adjustment:
                    img = self.adjust_image(img)
                if self.save_all_images:
                    self.save_image(img, label)
                else:
                    self.get_logger().warn("Sauvegarde désactivée (save_all_images=False).")
                ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.publisher.publish(ros_img)
            else:
                self.get_logger().error("Erreur lors du décodage de l'image capturée.")
        else:
            err = process.stderr.decode("utf-8", errors="replace") if process.stderr else "Erreur inconnue"
            self.get_logger().error(f"Erreur FFmpeg (code {process.returncode}) : {err[-600:]}")
            self.get_logger().error(f"Vérifiez : ping {self.ptz_host}, URL RTSP, ffmpeg -i rtsp://...")
  
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

    def save_image(self, img, label=""):
        """
        Sauvegarde l'image capturée avec un timestamp (label optionnel dans le nom).
        """
        try:
            if not os.path.exists(self.images_output_dir):
                os.makedirs(self.images_output_dir)
                self.get_logger().info(f"Dossier créé : {self.images_output_dir}")
            self.image_count += 1
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            suffix = f"_{label}" if label else ""
            image_filename = f"image_{self.image_count:04d}_{timestamp}{suffix}.jpg"
            image_path = os.path.join(self.images_output_dir, image_filename)
            if cv2.imwrite(image_path, img):
                self.get_logger().info(f"Image sauvegardée : {image_path} ({img.shape[1]}x{img.shape[0]})")
            else:
                self.get_logger().error(f"Échec cv2.imwrite : {image_path}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la sauvegarde : {e}")

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
