import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class PTZControllerVISCA(Node):
    """Contrôleur PTZ pour caméra Marshall CV-605 via protocole VISCA over IP
    
    Ce nœud gère :
    - Les commandes de mouvement continu (cmd_vel) via /ptz/cmd_vel
    - La commande Home (preset -1) via /ptz/preset
    """
    
    def __init__(self):
        super().__init__('ptz_controller')
        
        # Connexion VISCA
        self.sock = None
        self._connect_visca()
        
        # Subscriber pour les commandes de mouvement
        self.create_subscription(Twist, '/ptz/cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscriber pour les presets 
        self.create_subscription(Int32, '/ptz/preset', self.preset_callback, 10)
        
        self.get_logger().info("PTZ Controller démarré (cmd_vel + preset Home)")
    
    def _connect_visca(self):
        """Établit la connexion VISCA avec la caméra"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect(('192.168.5.163', 1259))
            self.get_logger().info('Connexion VISCA établie: 192.168.5.163:1259')
        except Exception as e:
            self.get_logger().error(f'Erreur de connexion VISCA: {e}')
            if self.sock:
                self.sock.close()
                self.sock = None
    
    def send_visca_command(self, command_bytes):
        """Envoie une commande VISCA avec tentative de reconnexion automatique"""
        # Si le socket est fermé, on tente de le rouvrir
        if self.sock is None:
            self.get_logger().warn("Socket non initialisé, tentative de reconnexion...")
            self._connect_visca()
            
        if not self.sock:
            self.get_logger().error('Échec critique : Impossible de joindre la caméra.')
            return False
            
        try:
            header = bytes([0x81]) # Directement 0x81 pour l'adresse 1
            terminator = bytes([0xFF])
            packet = header + command_bytes + terminator
            
            self.sock.send(packet)
            return True
                
        except (socket.error, Exception) as e:
            self.get_logger().error(f'Erreur lors de l\'envoi : {e}')
            # On nettoie le socket pour forcer la reconnexion au prochain essai
            if self.sock:
                self.sock.close()
            self.sock = None
            return False
    
    def cmd_vel_callback(self, msg):
        """Contrôle pan/tilt depuis un message Twist
        
        Mapping:
            - angular.z: Pan (rotation horizontale) - positif = droite, négatif = gauche
            - linear.y: Tilt (rotation verticale) - positif = haut, négatif = bas
        """
        if not self.sock:
            self.get_logger().warn("Socket VISCA non initialisé, tentative de reconnexion...")
            self._connect_visca()
            if not self.sock:
                self.get_logger().error("Impossible de se connecter à la caméra PTZ")
                return
            
        # Conversion Twist -> VISCA (Pan: 1-18, Tilt: 1-14)
        pan_speed = int(min(max(abs(msg.angular.z) * 18, 1), 18))
        tilt_speed = int(min(max(abs(msg.linear.y) * 14, 1), 14))
        
        # Déterminer la direction selon le format VISCA Marshall CV-605
        if abs(msg.angular.z) > 0.1 or abs(msg.linear.y) > 0.1:
            dir_bytes = self._get_direction_bytes(msg.angular.z, msg.linear.y)
            command = bytes([0x01, 0x06, 0x01, pan_speed, tilt_speed, dir_bytes[0], dir_bytes[1]])
            self.send_visca_command(command)
        else:
            # Stop
            command = bytes([0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03])
            self.send_visca_command(command)
    
    def _get_direction_bytes(self, pan_value, tilt_value):
        """Détermine les bytes de direction VISCA selon les valeurs pan/tilt"""
        if tilt_value > 0.1:  # Tilt Up
            if pan_value > 0.1:
                return [0x02, 0x01]  # Upright
            elif pan_value < -0.1:
                return [0x01, 0x01]  # Upleft
            else:
                return [0x03, 0x01]  # Up
        elif tilt_value < -0.1:  # Tilt Down
            if pan_value > 0.1:
                return [0x02, 0x02]  # Downright
            elif pan_value < -0.1:
                return [0x01, 0x02]  # Downleft
            else:
                return [0x03, 0x02]  # Down
        else:  # Pas de tilt
            if pan_value > 0.1:
                return [0x02, 0x03]  # Right
            elif pan_value < -0.1:
                return [0x01, 0x03]  # Left
            else:
                return [0x03, 0x03]  # Stop
    
    def preset_callback(self, msg):
        preset = msg.data
        
        if preset == -1:
            # HOME : On envoie juste le corps de la commande
            command = bytes([0x01, 0x06, 0x04]) 
            if self.send_visca_command(command):
                self.get_logger().info('PTZ: Home (Retour au centre)')

        elif preset == 0:
            # RESET : Calibration
            command = bytes([0x01, 0x06, 0x05])
            if self.send_visca_command(command):
                self.get_logger().info('PTZ: Reset (Calibration complète des moteurs)')

        else:
            # RAPPEL PRESET : 01 04 3F 02 [preset]
            command = bytes([0x01, 0x04, 0x3F, 0x02, preset])
            if self.send_visca_command(command):
                self.get_logger().info(f'PTZ: Rappel Preset {preset}')


def main(args=None):
    rclpy.init(args=args)
    controller = PTZControllerVISCA()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if controller.sock:
            controller.sock.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
