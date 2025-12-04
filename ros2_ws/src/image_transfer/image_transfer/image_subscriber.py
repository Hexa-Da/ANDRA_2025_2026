import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from ultralytics import YOLO
from geometry_msgs.msg import Point
import cv2
import numpy as np
import os
import time

class Noeud5(Node):
    def __init__(self):
        super().__init__('noeud_5')
        self.position_pub = self.create_publisher(Point, 'position_detectee', 10)
        self.position = None
        # Abonnement au topic 'photo_topic' pour recevoir les images
        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.listener_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.bridge = CvBridge()

        # Charger le modèle YOLO
        # model_path = '/home/techlab/dossier_oriana/package1/models/tags_model.pt'
        model_path = '/home/techlab/Documents/Projet\ ANDRA\ 2024-2025/ros2_ws/models/best.pt' 
        self.model = YOLO(model_path)

        # Création du dossier de stockage des images détectées
        self.output_dir = "images_detectees"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.get_logger().info('Nœud 5 démarré et prêt à recevoir les images.')
    
    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position

    def listener_callback(self, msg):
        """Récupère l'image, applique l'IA, publie si une étiquette est détectée"""
        try:
            # Convertir l'image ROS en image OpenCV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur de conversion d'image : {e}")
            return

        # Appliquer la détection
        detected, img_with_boxes = self.detect_tags(img)

        # Afficher et enregistrer l'image si une détection est faite
        if detected:
            self.get_logger().info("✅ Étiquette détectée.")
            
            # Sauvegarde de l'image avec timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(self.output_dir, f"detection_{timestamp}.jpg")
            cv2.imwrite(image_path, img_with_boxes)
            self.get_logger().info(f"🖼️ Image sauvegardée : {image_path}")
            self.get_logger().info(f"Voici la position du robot : {self.position}")
            # Publier la position détectée
            if self.position is not None:
                point_msg = Point()
                point_msg.x = self.position.x
                point_msg.y = self.position.y
                point_msg.z = self.position.z
                self.position_pub.publish(point_msg)
                self.get_logger().info("📡 Position publiée à un autre nœud.")
            
        else:
            self.get_logger().info("❌ Aucune étiquette détectée.")

    def detect_tags(self, img):
        """Applique le modèle YOLO et retourne l'image annotée"""
        try:
            results = self.model(img)
            img_with_boxes = img.copy()
            detected = False

            for result in results:
                if result.boxes is not None and len(result.boxes) > 0:
                    for detection in result.boxes:
                        # Correction pour récupérer les valeurs correctement
                        x1, y1, x2, y2 = map(int, detection.xyxy[0].cpu().numpy())
                        conf = detection.conf[0].item()
                        cls = int(detection.cls[0].item())
                        label = f"Classe {cls}: {conf:.2f}"

                        # Dessiner la bounding box
                        cv2.rectangle(img_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(img_with_boxes, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        detected = True  # Au moins une détection trouvée

            return detected, img_with_boxes
        except Exception as e:
            self.get_logger().error(f"Erreur dans detect_tags : {e}")
            return False, img

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

