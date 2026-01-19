import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
import tf2_ros

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Abonnement à l'odométrie (essayer d'abord filtered, puis odom_robot)
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Ou '/odom_robot' si EKF désactivé
            self.odom_callback,
            10
        )
        
        # Publication du Path
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        
        # NOUVEAU : Broadcaster TF pour publier odom -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialisation du Path
        self.path = Path()
        self.path.header.frame_id = 'odom'  # Frame de référence
        
        # Paramètres
        self.min_distance = 0.1  # Ajouter un point tous les 10 cm minimum
        
        self.get_logger().info('Nœud odom_to_path démarré. Publication du Path sur /robot_path et TF odom->base_link')
        
    def odom_callback(self, msg):
        """Callback appelé à chaque message d'odométrie"""
        current_pose = msg.pose.pose
        current_position = current_pose.position
        
        # NOUVEAU : Publier la transformation TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Copier la position et orientation depuis l'odométrie
        t.transform.translation.x = current_position.x
        t.transform.translation.y = current_position.y
        t.transform.translation.z = current_position.z
        
        t.transform.rotation.x = current_pose.orientation.x
        t.transform.rotation.y = current_pose.orientation.y
        t.transform.rotation.z = current_pose.orientation.z
        t.transform.rotation.w = current_pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        
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
            pose_stamped.pose = current_pose
            self.path.poses.append(pose_stamped)
            self.path.header.stamp = self.get_clock().now().to_msg()
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