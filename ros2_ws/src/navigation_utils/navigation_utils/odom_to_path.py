import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Paramètres ROS : distance min entre points + taille max du Path (FIFO)
        # pour éviter une croissance illimitée en mission longue.
        self.declare_parameter('min_distance', 0.1)
        self.declare_parameter('max_path_size', 5000)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.max_path_size = max(1, int(self.get_parameter('max_path_size').value))
        
        self.get_logger().info(
            f'Nœud odom_to_path démarré (min_distance={self.min_distance:.2f}m, '
            f'max_path_size={self.max_path_size}).'
        )
        
    def odom_callback(self, msg):
        """Callback appelé à chaque message d'odométrie"""
        current_pose = msg.pose.pose
        current_position = current_pose.position
        
        # Code existant pour le Path
        # Calculer la distance depuis le dernier point
        if len(self.path.poses) > 0:
            last_pose = self.path.poses[-1].pose.position
            distance = math.sqrt(
                (current_position.x - last_pose.x)**2 + 
                (current_position.y - last_pose.y)**2
            )
        else:
            distance = float('inf')
        
        # Ajouter un point si on a bougé assez
        if distance > self.min_distance or len(self.path.poses) == 0:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose = current_pose

            self.path.poses.append(pose_stamped)
            # Bornage FIFO du Path pour éviter la croissance illimitée en mission longue.
            if len(self.path.poses) > self.max_path_size:
                excess = len(self.path.poses) - self.max_path_size
                del self.path.poses[:excess]
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path.header.frame_id = 'odom'
            self.path_publisher.publish(self.path)
            
            # Log périodique (tous les 50 points pour éviter le spam)
            if len(self.path.poses) % 50 == 0:
                self.get_logger().info(f'Path mis à jour : {len(self.path.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()