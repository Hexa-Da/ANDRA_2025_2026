import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Noeud4(Node):
    def __init__(self):
        super().__init__('noeud_4')

        self.subscription = self.create_subscription(
            Image,
            'photo_topic',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convertir le message ROS en image OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Afficher l'image reçue (optionnel)
        cv2.imshow("Photo reçue", image)
        cv2.waitKey(1)

        # Afficher un message dans le terminal
        self.get_logger().info("Photo bien reçue")

def main(args=None):
    rclpy.init(args=args)
    noeud_4 = Noeud4()
    try:
        rclpy.spin(noeud_4)
    except KeyboardInterrupt:
        pass
    finally:
        noeud_4.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Fermer la fenêtre d'affichage

if __name__ == '__main__':
    main()

