#!/usr/bin/env python3
"""
Nœud ROS2 central pour la gestion des presets PTZ.

Ce nœud gère TOUTES les opérations de presets PTZ :
- Appel des presets via /ptz/preset
- Enregistrement des presets via /ptz/save_preset
- Publication des infos presets sur /ptz/presets_info

Presets définis:
- Preset 0: Position centrale (Home)
- Preset 1: 90° à gauche
- Preset 2: Vers le haut
- Preset 3: 90° à droite

Commandes spéciales:
- -1: Home (retour position centrale)
- -2: Reset caméra
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import json
import time
import socket


class PTZPresetsManager(Node):
    """Nœud ROS2 central pour gérer les presets PTZ"""
    
    # Définition des presets (configuration centralisée)
    PRESETS = {
        0: {"name": "Centre", "description": "Position centrale (Home)"},
        1: {"name": "Gauche", "description": "90° à gauche"},
        2: {"name": "Haut", "description": "Vers le haut"},
        3: {"name": "Droite", "description": "90° à droite"},
    }
    
    # Configuration des positions absolues des presets (pan/tilt en degrés)
    # Pan: -170° à +170° (négatif = gauche, positif = droite)
    # Tilt: -20° à +90° (négatif = bas, positif = haut)
    PRESET_POSITIONS = {
        0: {"pan": 0, "tilt": 0, "use_home": True},      # Centre (Home)
        1: {"pan": -90, "tilt": 0, "use_home": False},   # 90° à gauche
        2: {"pan": 0, "tilt": 45, "use_home": False},    # Vers le haut
        3: {"pan": 90, "tilt": 0, "use_home": False},    # 90° à droite
    }
    
    def __init__(self):
        super().__init__('ptz_presets_manager')
        
        # Configuration VISCA
        self.cam_ip = '192.168.5.163'
        self.cam_port = 1259
        self.sock = None
        
        # Publishers
        self.presets_info_pub = self.create_publisher(String, '/ptz/presets_info', 10)
        self.ptz_cmd_vel_pub = self.create_publisher(Twist, '/ptz/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Int32, '/ptz/preset', self.preset_callback, 10)
        self.create_subscription(Int32, '/ptz/save_preset', self.save_preset_callback, 10)
        
        # Timer pour publier les infos des presets périodiquement
        self.create_timer(5.0, self.publish_presets_info)
        
        # Flag pour éviter les tentatives multiples de connexion
        self._connection_attempted = False
        
        # Timer pour tenter la connexion VISCA avec retry (après un court délai)
        self.create_timer(2.0, self._try_connect_visca_once)
        
        self.get_logger().info("=== PTZ Presets Manager démarré ===")
        self.get_logger().info(f"Presets disponibles: {list(self.PRESETS.keys())}")
        self.get_logger().info("Commandes: -1=Home, -2=Reset, 0-127=Preset")
        self.get_logger().info("Tentative de connexion VISCA dans 2 secondes...")
        self.publish_presets_info()
    
    def _try_connect_visca_once(self):
        """Tente une seule fois de se connecter à VISCA (appelé par timer)"""
        if not self._connection_attempted and self.sock is None:
            self._connection_attempted = True
            self._connect_visca()
    
    def _connect_visca(self, retry_count=3, retry_delay=1.0):
        """Établit la connexion VISCA avec la caméra (avec retry)"""
        for attempt in range(retry_count):
            try:
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None
                
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((self.cam_ip, self.cam_port))
                self.get_logger().info(f'Connexion VISCA établie: {self.cam_ip}:{self.cam_port}')
                return True
            except Exception as e:
                if attempt < retry_count - 1:
                    self.get_logger().warn(f'Tentative {attempt + 1}/{retry_count} de connexion VISCA échouée: {e}. Nouvelle tentative dans {retry_delay}s...')
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(f'Impossible de se connecter à VISCA après {retry_count} tentatives: {e}')
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None
        
        self.get_logger().warn('Le nœud continuera de fonctionner mais les commandes VISCA ne seront pas envoyées jusqu\'à reconnexion')
        return False
    
    def send_visca_command(self, command_bytes):
        """Envoie une commande VISCA à la caméra"""
        if not self.sock:
            self.get_logger().warn("Socket VISCA non connecté, tentative de reconnexion...")
            self._connect_visca()
            if not self.sock:
                self.get_logger().error("Impossible de se connecter à la caméra PTZ")
                return False
        
        try:
            header = bytes([0x80 + 1])  # Adresse caméra = 1
            terminator = bytes([0xFF])
            packet = header + command_bytes + terminator
            
            self.sock.send(packet)
            return True
        except Exception as e:
            self.get_logger().error(f'Erreur VISCA: {e}')
            if self.sock:
                self.sock.close()
                self.sock = None
            return False
    
    def publish_presets_info(self):
        """Publie les informations des presets sur le topic"""
        msg = String()
        msg.data = json.dumps(self.PRESETS)
        self.presets_info_pub.publish(msg)
    
    def preset_callback(self, msg):
        """Callback pour appeler un preset ou une commande spéciale"""
        preset = msg.data
        self.get_logger().info(f"Commande preset reçue: {preset}")
        
        if preset == -1:
            # Home: retour à la position centrale
            command = bytes([0x01, 0x06, 0x04])
            if self.send_visca_command(command):
                self.get_logger().info('PTZ: Home (position centrale)')
        elif preset == -2:
            # Reset: réinitialisation
            command = bytes([0x01, 0x06, 0x05])
            if self.send_visca_command(command):
                self.get_logger().info('PTZ: Reset')
        elif 0 <= preset <= 127:
            # Si c'est un preset prédéfini avec position, utiliser la position absolue
            if preset in self.PRESET_POSITIONS:
                position = self.PRESET_POSITIONS[preset]
                name = self.PRESETS.get(preset, {}).get('name', f'Preset {preset}')
                self.get_logger().info(f'PTZ: Preset {preset} ({name}) - position absolue')
                
                if position['use_home']:
                    # Utiliser la commande Home VISCA
                    command = bytes([0x01, 0x06, 0x04])
                    if self.send_visca_command(command):
                        self.get_logger().info(f'PTZ: Preset {preset} ({name}) - Home')
                else:
                    # Envoyer la position absolue VISCA
                    if self.set_absolute_position(position['pan'], position['tilt']):
                        self.get_logger().info(f'PTZ: Preset {preset} ({name}) - Pan={position["pan"]}°, Tilt={position["tilt"]}°')
            else:
                # Preset personnalisé : essayer de rappeler le preset VISCA
                command = bytes([0x01, 0x04, 0x3F, 0x02, preset])
                if self.send_visca_command(command):
                    name = self.PRESETS.get(preset, {}).get('name', f'Preset {preset}')
                    self.get_logger().info(f'PTZ: Preset {preset} ({name}) appelé via VISCA')
        else:
            self.get_logger().warn(f'Preset invalide: {preset} (doit être -2, -1, ou 0-127)')
    
    def save_preset_callback(self, msg):
        """Callback pour enregistrer un preset"""
        preset_num = msg.data
        
        if preset_num < 0 or preset_num > 127:
            self.get_logger().warn(f"Numéro de preset invalide: {preset_num}")
            return
        
        preset_info = self.PRESETS.get(preset_num, {"name": f"Preset {preset_num}"})
        self.get_logger().info(f"Enregistrement du preset {preset_num}: {preset_info.get('name', '')}")
        
        # Si c'est un preset prédéfini avec position, positionner d'abord la caméra
        if preset_num in self.PRESET_POSITIONS:
            self.setup_single_preset(preset_num)
        else:
            # Enregistrer la position actuelle
            self.save_preset_visca(preset_num)
    
    def setup_single_preset(self, preset_num):
        """Configure un seul preset (positionne puis enregistre)"""
        position = self.PRESET_POSITIONS.get(preset_num)
        if not position:
            return
        
        # D'abord, retourner au centre
        self.get_logger().info("Retour au centre...")
        command = bytes([0x01, 0x06, 0x04])  # Home
        self.send_visca_command(command)
        time.sleep(2.0)
        
        if not position['use_home']:
            # Positionner la caméra à la position absolue
            self.get_logger().info(f"Positionnement: pan={position['pan']}°, tilt={position['tilt']}°")
            if self.set_absolute_position(position['pan'], position['tilt']):
                time.sleep(2.0)  # Attendre que la caméra atteigne la position
        
        # Enregistrer le preset via VISCA
        if self.save_preset_visca(preset_num):
            self.get_logger().info(f"Preset {preset_num} enregistré avec succès")
        else:
            self.get_logger().error(f"Échec enregistrement preset {preset_num}")
        
        # Retour au centre
        command = bytes([0x01, 0x06, 0x04])  # Home
        self.send_visca_command(command)
    
    def set_absolute_position(self, pan_deg, tilt_deg):
        """Positionne la caméra à une position absolue (pan/tilt en degrés)
        
        Args:
            pan_deg: Pan en degrés (-170 à +170, négatif=gauche, positif=droite)
            tilt_deg: Tilt en degrés (-20 à +90, négatif=bas, positif=haut)
        
        Returns:
            bool: True si la commande a été envoyée avec succès
        """
        # Convertir les degrés en valeurs VISCA (16 bits: 0x0000 à 0xFFFF)
        # Pan: -170° à +170° -> 0x0000 à 0xFFFF (centre = 0x8000 = 32768)
        pan_range = 340  # 170 - (-170)
        pan_value = int(((pan_deg + 170) / pan_range) * 65535)
        pan_value = max(0, min(65535, pan_value))
        
        # Tilt: -20° à +90° -> 0x0000 à 0xFFFF
        tilt_range = 110  # 90 - (-20)
        tilt_value = int(((tilt_deg + 20) / tilt_range) * 65535)
        tilt_value = max(0, min(65535, tilt_value))
        
        # Convertir en bytes VISCA (4 bytes par position: p1 p2 p3 p4)
        # Format VISCA: chaque position sur 4 bytes (16 bits)
        # p1 = (value >> 12) & 0x0F (4 bits haut)
        # p2 = (value >> 8) & 0x0F (4 bits milieu-haut)
        # p3 = (value >> 4) & 0x0F (4 bits milieu-bas)
        # p4 = value & 0x0F (4 bits bas)
        pan_p1 = (pan_value >> 12) & 0x0F
        pan_p2 = (pan_value >> 8) & 0x0F
        pan_p3 = (pan_value >> 4) & 0x0F
        pan_p4 = pan_value & 0x0F
        
        tilt_p1 = (tilt_value >> 12) & 0x0F
        tilt_p2 = (tilt_value >> 8) & 0x0F
        tilt_p3 = (tilt_value >> 4) & 0x0F
        tilt_p4 = tilt_value & 0x0F
        
        # Commande VISCA: Pan/Tilt Absolute Position
        # Format: 0x01 0x06 0x02 pan_speed tilt_speed pan_p1 pan_p2 pan_p3 pan_p4 tilt_p1 tilt_p2 tilt_p3 tilt_p4
        # Vitesses: 0x01-0x18 pour pan, 0x01-0x14 pour tilt
        pan_speed = 0x0C  # Vitesse moyenne (12)
        tilt_speed = 0x0A  # Vitesse moyenne (10)
        
        command = bytes([
            0x01, 0x06, 0x02,
            pan_speed, tilt_speed,
            pan_p1, pan_p2, pan_p3, pan_p4,
            tilt_p1, tilt_p2, tilt_p3, tilt_p4
        ])
        
        return self.send_visca_command(command)
    
    def save_preset_visca(self, preset_num):
        """Enregistre la position actuelle comme preset via VISCA"""
        command = bytes([0x01, 0x04, 0x3F, 0x01, preset_num])
        return self.send_visca_command(command)


def main(args=None):
    rclpy.init(args=args)
    node = PTZPresetsManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock:
            node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
