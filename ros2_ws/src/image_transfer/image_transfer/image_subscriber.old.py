import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class Noeud5(Node):
    def __init__(self):
        super().__init__('noeud_5')

        # Abonnement au topic 'photo_topic' pour recevoir les images
        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.listener_callback,
            10
        )

        # Publication sur le topic 'detection_status'
        self.detection_publisher = self.create_publisher(
            Bool,
            'detection_status',
            10
        )

        self.bridge = CvBridge()

        # Charger le modèle YOLO
        model_path = '/home/techlab/dossier_oriana/package1/models/tags_model.pt'
        self.model = YOLO(model_path)

        self.get_logger().info('Nœud 5 démarré et prêt à recevoir les images.')

    def listener_callback(self, msg):
        """Récupère l'image, applique l'IA, publie si une étiquette est détectée"""
        try:
            # Convertir l'image ROS en image OpenCV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur de conversion d'image : {e}")
            return

        # Appliquer la détection
        detected = self.detect_tags(img)

        # Créer et publier le message Booléen
        detection_msg = Bool()
        detection_msg.data = bool(detected)
        self.detection_publisher.publish(detection_msg)

        # Afficher l'état de la détection
        if detected:
            self.get_logger().info("✅ Étiquette détectée.")
        else:
            self.get_logger().info("❌ Aucune étiquette détectée.")

    def detect_tags(self, img):
        try:
            results = self.model(img)

            for result in results:
                if result.boxes is not None and len(result.boxes) > 0:
                    for detection in result.boxes:
                        if len(detection.xyxy[0]) >= 4:
                            x1, y1, x2, y2 = detection.xyxy[0][:4]
                            print(f"Détection de l'objet dans l'image : x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                    return True  # On a trouvé au moins une détection
            return False
        except Exception as e:
            self.get_logger().error(f"Erreur dans detect_tags : {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Noeud5()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

