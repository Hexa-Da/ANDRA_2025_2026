import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2

class Noeud5(Node):
    def __init__(self):
        super().__init__('noeud_5')
        # Abonnement au topic 'photo_topic' du nœud_3
        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Charger le modèle YOLO
        model_path = '/home/techlab/Documents/Projet\ ANDRA\ 2024-2025/ros2_ws/models/best.pt'  
        self.model = YOLO(model_path)  # Charger le modèle YOLO
        self.get_logger().info('Nœud 5 démarré et prêt à recevoir les images.')

    def listener_callback(self, msg):
        """Récupère l'image du nœud_3, applique l'IA et détecte les étiquettes"""
        try:
            # Convertir l'image ROS en image OpenCV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")
            return

        # Appliquer l'IA pour détecter les étiquettes dans l'image
        detected = self.detect_tags(img)

        # Vérifier si des étiquettes ont été détectées et afficher le message correspondant
        if detected:
            self.get_logger().info("Étiquette trouvée")
        else:
            self.get_logger().info("Étiquette non trouvée")

    def detect_tags(self, img):
        """Applique le modèle YOLO pour détecter des étiquettes dans l'image"""
        # Appliquer le modèle YOLO à l'image
        results = self.model(img)  # Passe l'image au modèle YOLO

        # Vérifie si des détections sont présentes
        if len(results[0].boxes) > 0:  # Si des objets sont détectés
            annotated_frame = results[0].plot()  # Annoter l'image (plot() peut être utilisé pour la visualisation)
            cv2.imshow("Detection", annotated_frame)  # Afficher l'image annotée

            # Actions basées sur les résultats YOLO (optionnelles)
            for result in results[0].boxes:  # Pour chaque détection
                x1, y1, x2, y2, conf, cls = result.xyxy[0]  # Coordonnées de la boîte, confiance et classe
                self.get_logger().info(f"Objet détecté : Classe={int(cls)}, Confiance={conf:.2f}")

            return True  # Des objets ont été détectés
        else:
            self.get_logger().info("Aucune détection trouvée.")
            return False  # Pas d'objets détectés


def main(args=None):
    rclpy.init(args=args)
    noeud_5 = Noeud5()

    try:
        rclpy.spin(noeud_5)
    except KeyboardInterrupt:
        pass
    finally:
        noeud_5.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

