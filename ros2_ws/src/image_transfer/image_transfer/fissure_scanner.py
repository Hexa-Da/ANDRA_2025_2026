import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import time

class FissureScanner(Node):
    def __init__(self):
        super().__init__('fissure_scanner')

        # Publishers pour contrôler les autres nœuds
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ptz_preset_pub = self.create_publisher(Int32, '/ptz/preset', 10)
        self.trigger_pub = self.create_publisher(Bool, 'trigger_capture', 10)

        self.get_logger().info("Nœud FissureScanner prêt. Appuyez sur Entrée dans le terminal pour lancer la séquence.")
        
        # On lance une boucle pour attendre l'input utilisateur sans bloquer ROS
        self.create_timer(0.1, self.check_input)

    def stop_robot(self):
        """Envoie une commande d'arrêt total au robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("🛑 Arrêt du robot demandé.")

    def trigger_photo(self):
        """Demande à image_publisher de capturer une image"""
        msg = Bool()
        msg.data = True
        self.trigger_pub.publish(msg)
        self.get_logger().info("📸 Photo déclenchée !")

    def move_cam_and_wait(self, preset_id, wait_time=3.0):
        """Déplace la caméra et attend qu'elle soit stable"""
        msg = Int32()
        msg.data = preset_id
        self.ptz_preset_pub.publish(msg)
        self.get_logger().info(f"🔄 Déplacement vers Preset {preset_id}...")
        
        # On attend le temps spécifié pour la stabilisation mécanique
        time.sleep(wait_time)

    def run_sequence(self):
        """Séquence : Stop -> Gauche -> Haut -> Droite -> Home"""
        self.get_logger().info("=== DÉBUT DE LA SÉQUENCE D'INSPECTION ===")
        
        # 1. Arrêter le robot
        self.stop_robot()
        time.sleep(2.0) # Petite pause pour l'inertie du robot

        # 2. Inspection GAUCHE
        self.move_cam_and_wait(1, wait_time=1.0) # Supposons Preset 1 = Gauche
        self.trigger_photo()
        time.sleep(15.0)


        stop_msg = Twist() 
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("🛑 Arrêt forcé des moteurs PTZ")

        time.sleep(5.0)


        # 3. Inspection HAUT
        self.move_cam_and_wait(2, wait_time=1.0) # Supposons Preset 2 = Haut
        self.trigger_photo()
        time.sleep(15.0)


        stop_msg = Twist() 
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("🛑 Arrêt forcé des moteurs PTZ")

        time.sleep(5.0)





        # 4. Inspection DROITE
        self.move_cam_and_wait(3, wait_time=1.0) # Supposons Preset 3 = Droite
        self.trigger_photo()
        time.sleep(15.0)


        stop_msg = Twist() 
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("🛑 Arrêt forcé des moteurs PTZ")

        time.sleep(5.0)



        # 5. Retour au centre
        self.move_cam_and_wait(-1, wait_time=1.0) # -1 = Home dans ton ptz_controller
        
        self.get_logger().info("=== SÉQUENCE TERMINÉE ===")

    def check_input(self):
        # Cette fonction peut être améliorée pour détecter une touche spécifique
        # Pour l'instant, elle attend un 'Entrée' dans le terminal pour tester
        input("Appuyez sur [Entrée] pour lancer l'inspection...")
        self.run_sequence()

def main(args=None):
    rclpy.init(args=args)
    node = FissureScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()