import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class PTZControllerVISCA(Node):
    """Contrôleur PTZ pour caméra Marshall CV-605 via protocole VISCA over IP"""
    
    def __init__(self):
        super().__init__('ptz_controller')
        
        # Connexion VISCA
        self.sock = None
        self._connect_visca()
        
        # Subscribers
        self.create_subscription(Twist, '/ptz/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, '/ptz/preset', self.preset_callback, 10)
    
    def _connect_visca(self):
        """Établit la connexion VISCA avec la caméra"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect(('192.168.5.163', 1259))
            self.get_logger().info('Connexion VISCA établie: 192.168.5.163:1259, Address: 1')
        except Exception as e:
            self.get_logger().error(f'Erreur de connexion VISCA: {e}')
            if self.sock:
                self.sock.close()
                self.sock = None
    
    def send_visca_command(self, command_bytes):
        """Envoie une commande VISCA à la caméra (mode fire-and-forget)
        
        Args:
            command_bytes: Bytes de la commande VISCA (sans header ni terminator)
            
        Returns:
            bool: True si l'envoi réussit, False sinon
        """
        if not self.sock:
            self.get_logger().error('Socket VISCA non initialisé')
            return False
            
        try:
            # Format VISCA: Header (0x80 + address) + Command + Terminator (0xFF)
            header = bytes([0x80 + 1])
            terminator = bytes([0xFF])
            packet = header + command_bytes + terminator
            
            self.sock.send(packet)
            return True
                
        except Exception as e:
            self.get_logger().error(f'Erreur VISCA: {e}')
            return False
    
    def cmd_vel_callback(self, msg):
        """Contrôle pan/tilt depuis un message Twist
        
        Mapping:
            - angular.z: Pan (rotation horizontale) - positif = droite, négatif = gauche
            - linear.y: Tilt (rotation verticale) - positif = haut, négatif = bas
        """
        if not self.sock:
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
        """Détermine les bytes de direction VISCA selon les valeurs pan/tilt
        
        Returns:
            list: [byte1, byte2] selon le format Marshall CV-605
        """
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
        """Gère les presets et commandes spéciales
        
        Valeurs spéciales:
            -1: Home (position centrale)
            -2: Reset
            0-127: Preset VISCA standard
        """
        if not self.sock:
            return
            
        preset = msg.data
        
        if preset == -1:
            # Home: retour à la position centrale
            command = bytes([0x01, 0x06, 0x04])
            self.send_visca_command(command)
            self.get_logger().info('PTZ: Home (position centrale)')
        elif preset == -2:
            # Reset: réinitialisation
            command = bytes([0x01, 0x06, 0x05])
            self.send_visca_command(command)
            self.get_logger().info('PTZ: Reset')
        elif 0 <= preset <= 127:
            # Preset VISCA standard
            command = bytes([0x01, 0x04, 0x3F, 0x02, preset])
            self.send_visca_command(command)
            self.get_logger().info(f'Preset {preset} appelé')
        else:
            self.get_logger().warn(f'Preset invalide: {preset} (doit être 0-127, -1 pour Home, -2 pour Reset)')

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