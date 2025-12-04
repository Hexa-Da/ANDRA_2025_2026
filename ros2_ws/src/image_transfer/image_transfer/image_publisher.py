import rclpy
from rclpy.node import Node
import subprocess
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Noeud3(Node):
    def __init__(self):
        super().__init__('noeud_3')
        self.publisher = self.create_publisher(Image, 'photo_topic', 10)
        self.bridge = CvBridge()

        # Capture une image toutes les 20 secondes
        self.timer = self.create_timer(20, self.capture_and_publish)

    def capture_and_publish(self):
        self.get_logger().info("Capture d'une image depuis la caméra PTZ...")

        # Exécute FFmpeg et récupère l'image directement en mémoire
        ffmpeg_command = [
            "ffmpeg", "-i", "rtsp://admin:admin@192.168.5.163:554/live/av0",
            "-vframes", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1"
        ]

        process = subprocess.run(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        if process.returncode == 0:
            # Convertir le flux binaire en image OpenCV
            np_arr = np.frombuffer(process.stdout, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                # Convertir l'image en message ROS 2
                ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.publisher.publish(ros_img)
                self.get_logger().info("Image capturée et publiée avec succès.")
            else:
                self.get_logger().error("Erreur lors du décodage de l'image capturée.")
        else:
            self.get_logger().error("Erreur lors de la capture avec FFmpeg.")

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
