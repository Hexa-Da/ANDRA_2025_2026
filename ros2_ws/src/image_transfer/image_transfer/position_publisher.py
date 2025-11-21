import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')

        self.detection_status = False
        self.position = None

        # Abonnement au topic de détection
        self.detection_sub = self.create_subscription(
            Bool,
            'detection_status',
            self.detection_callback,
            10
        )

        # Abonnement à la position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Timer pour affichage régulier si une détection est présente
        self.timer = self.create_timer(1.0, self.check_and_print_position)

    def detection_callback(self, msg: Bool):
        self.detection_status = msg.data
        if self.detection_status:
            self.get_logger().info("Signal de détection reçu.")

    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position

    def check_and_print_position(self):
        if self.detection_status and self.position:
            x = self.position.x
            y = self.position.y
            z = self.position.z
            self.get_logger().info(f"Position détectée : x={x}, y={y}, z={z}")
            # Réinitialiser le flag pour éviter de répéter
            self.detection_status = False
        elif not self.position:
            self.get_logger().info("En attente des données de position...")

def main(args=None):
    rclpy.init(args=args)
    node = PositionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

