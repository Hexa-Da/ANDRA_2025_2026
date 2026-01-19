import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        # Create a subscription to the 'odom' topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Topic name (usually '/odom')
            self.odom_callback,
            10  # QoS (Quality of Service) depth
        )
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(1.0, self.print_position)  # 1.0s interval

    def odom_callback(self, msg: Odometry):
        # Extract the position (x, y, z) from the Odometry message
        self.position = msg.pose.pose.position

    def print_position(self):
        # Print the position every 1 second if available
        if self.position:
            x = self.position.x
            y = self.position.y
            z = self.position.z
            self.get_logger().info(f"Position - x: {x}, y: {y}, z: {z}")
        else:
            self.get_logger().info("Waiting for odometry data...")

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()

    # Spin the node to keep receiving messages
    rclpy.spin(odometry_subscriber)

    # Cleanup
    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

