#!/usr/bin/env python3
"""
Script d'automatisation pour faire avancer le robot de 1m, 
arrêter pendant 30 secondes avec captures PTZ, puis repartir en boucle.

Ce script utilise les nœuds ROS2 existants :
- ptz_controller : via /ptz/preset pour recentrer (Home) et /ptz/cmd_vel pour les mouvements
- image_publisher : via /trigger_capture pour déclencher les captures
- odometry/filtered : pour mesurer la distance parcourue
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32, String
import time
import json


class RobotSequence(Node):
    """Nœud ROS2 pour automatiser une séquence de mouvement et captures PTZ en boucle"""
    
    def __init__(self):
        super().__init__('robot_sequence')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ptz_preset_pub = self.create_publisher(Int32, '/ptz/preset', 10)
        self.ptz_cmd_vel_pub = self.create_publisher(Twist, '/ptz/cmd_vel', 10)
        self.trigger_capture_pub = self.create_publisher(Empty, '/trigger_capture', 10)
        
        # Subscriber pour l'odométrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Subscriber pour les presets PTZ (depuis ptz_presets_manager)
        self.presets_sub = self.create_subscription(
            String,
            '/ptz/presets_info',
            self.presets_callback,
            10
        )
        self.ptz_presets = None  # Dictionnaire des presets reçus
        
        # État du robot
        self.initial_position = None
        self.current_position = None
        self.state = 'WAITING_START'  # WAITING_START, MOVING, STOPPED, PTZ_MOVING, WAITING_STABLE, RESETTING
        
        # Paramètres
        self.target_distance = 1.0  # 1 mètre
        self.stop_duration = 30.0  # 30 secondes
        self.robot_speed = 0.3  # m/s (vitesse de déplacement)
        
        # Configuration PTZ - Utilisation des presets VISCA (lus depuis ptz_presets_manager)
        self.use_presets = True  # Utiliser les presets au lieu des mouvements continus
        self.ptz_preset_wait = 2.0  # Durée d'attente après appel d'un preset pour stabilisation
        self.ptz_move_90_duration = 5.0  # Durée pour atteindre 90° (si use_presets=False)
        self.ptz_move_up_duration = 2.5  # Durée pour aller vers le haut (si use_presets=False)
        self.ptz_stable_duration = 0.5  # Durée d'attente après mouvement pour stabilisation
        self.ptz_home_wait = 2.5  # Durée d'attente après Home pour recentrage
        
        # Mapping des presets par nom (sera mis à jour depuis ptz_presets_manager)
        self.preset_ids = {
            'centre': 0,
            'gauche': 1,
            'haut': 2,
            'droite': 3,
        }
        
        # Timer pour la séquence
        self.timer = self.create_timer(0.1, self.sequence_loop)  # 10 Hz
        
        self.get_logger().info("=== Séquence robot démarrée (mode boucle) ===")
        self.get_logger().info(f"Distance cible: {self.target_distance}m")
        self.get_logger().info(f"Durée d'arrêt: {self.stop_duration}s")
        self.get_logger().info("Appuyez sur Ctrl+C pour arrêter la boucle")
        
        # Temps de démarrage et gestion des états
        self.start_time = None
        self.stop_start_time = None
        self.ptz_move_start_time = None
        self.capture_index = 0
        self.ptz_step = 'idle'
        self.sequence_count = 0
        
    def reset_sequence(self):
        """Réinitialise la séquence pour une nouvelle itération"""
        self.initial_position = None
        self.current_position = None
        self.state = 'WAITING_START'
        self.capture_index = 0
        self.ptz_step = 'idle'
        self.sequence_count += 1
        self.get_logger().info(f"--- Début séquence #{self.sequence_count} ---")
        
    def odom_callback(self, msg):
        """Callback pour recevoir l'odométrie"""
        self.current_position = msg.pose.pose.position
        
        # Initialiser la position de départ au premier message
        if self.initial_position is None and self.state == 'WAITING_START':
            self.initial_position = (
                self.current_position.x,
                self.current_position.y
            )
            self.state = 'MOVING'
            self.start_time = time.time()
            self.get_logger().info(f"Position initiale enregistrée: ({self.initial_position[0]:.2f}, {self.initial_position[1]:.2f})")
            self.get_logger().info("Démarrage du mouvement...")
    
    def presets_callback(self, msg):
        """Callback pour recevoir la liste des presets depuis ptz_presets_manager"""
        try:
            presets_data = json.loads(msg.data)
            self.ptz_presets = presets_data
            
            # Mettre à jour le mapping des presets par nom
            for preset_id, info in presets_data.items():
                name_lower = info['name'].lower()
                self.preset_ids[name_lower] = int(preset_id)
            
            if not hasattr(self, '_presets_logged'):
                self.get_logger().info(f"Presets PTZ reçus: {list(self.ptz_presets.keys())}")
                self._presets_logged = True
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Erreur décodage presets: {e}")
    
    def get_preset_id(self, name):
        """Retourne l'ID d'un preset par son nom"""
        return self.preset_ids.get(name.lower(), -1)
    
    def get_preset_name(self, preset_id):
        """Retourne le nom d'un preset par son ID"""
        if self.ptz_presets and str(preset_id) in self.ptz_presets:
            return self.ptz_presets[str(preset_id)]['name']
        return f"Preset {preset_id}"
    
    def calculate_distance(self, pos1, pos2):
        """Calcule la distance euclidienne entre deux positions"""
        return ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5
    
    def move_robot(self, linear_x, angular_z=0.0):
        """Publie une commande de vitesse pour le robot"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Arrête le robot"""
        self.move_robot(0.0, 0.0)
    
    def move_ptz(self, pan, tilt):
        """Publie une commande de mouvement pour la PTZ"""
        twist = Twist()
        twist.angular.z = float(pan)
        twist.linear.y = float(tilt)
        self.ptz_cmd_vel_pub.publish(twist)
    
    def stop_ptz(self):
        """Arrête le mouvement de la PTZ"""
        self.move_ptz(0.0, 0.0)
    
    def set_ptz_home(self):
        """Utilise le preset Home pour recentrer la PTZ"""
        msg = Int32()
        msg.data = -1  # -1 = Home (position centrale)
        self.ptz_preset_pub.publish(msg)
        self.get_logger().info("PTZ: Commande Home (position centrale)")
    
    def start_ptz_movement(self, pan, tilt, duration):
        """Démarre un mouvement PTZ (utilisé si use_presets=False)"""
        self.ptz_move_start_time = time.time()
        self.ptz_move_duration = duration
        self.move_ptz(pan, tilt)
        self.state = 'PTZ_MOVING'
    
    def call_ptz_preset(self, preset_num):
        """Appelle un preset PTZ"""
        msg = Int32()
        msg.data = preset_num
        self.ptz_preset_pub.publish(msg)
        preset_name = self.get_preset_name(preset_num)
        self.get_logger().info(f"PTZ: Appel du preset {preset_num} ({preset_name})")
        self.state = 'WAITING_STABLE'
        self.stable_start_time = time.time()
    
    def trigger_capture(self, description=""):
        """Déclenche une capture via le topic /trigger_capture"""
        msg = Empty()
        self.trigger_capture_pub.publish(msg)
        if description:
            self.get_logger().info(f"Capture déclenchée: {description}")
    
    def sequence_loop(self):
        """Boucle principale de la séquence"""
        if self.state == 'WAITING_START':
            return
        
        elif self.state == 'MOVING':
            if self.current_position is None or self.initial_position is None:
                return
            
            # Calculer la distance parcourue
            current_pos = (self.current_position.x, self.current_position.y)
            distance = self.calculate_distance(self.initial_position, current_pos)
            
            if distance >= self.target_distance:
                # Distance atteinte, arrêter le robot
                self.stop_robot()
                self.state = 'STOPPED'
                self.stop_start_time = time.time()
                self.capture_index = 0
                self.get_logger().info(f"Distance de {distance:.2f}m atteinte. Arrêt du robot.")
                self.get_logger().info(f"Attente de {self.stop_duration}s pour les captures PTZ...")
            else:
                # Continuer à avancer
                self.move_robot(self.robot_speed)
        
        elif self.state == 'PTZ_MOVING':
            # Vérifier si le mouvement PTZ est terminé
            if self.ptz_move_start_time is not None:
                elapsed = time.time() - self.ptz_move_start_time
                if elapsed >= self.ptz_move_duration:
                    # Mouvement terminé, arrêter la PTZ et attendre la stabilisation
                    self.stop_ptz()
                    self.state = 'WAITING_STABLE'
                    self.stable_start_time = time.time()
        
        elif self.state == 'WAITING_STABLE':
            # Attendre la stabilisation après le mouvement PTZ ou appel de preset
            wait_duration = self.ptz_preset_wait if self.use_presets else self.ptz_stable_duration
            if time.time() - self.stable_start_time >= wait_duration:
                # Stabilisation terminée, retourner à STOPPED
                self.state = 'STOPPED'
        
        elif self.state == 'STOPPED':
            elapsed = time.time() - self.stop_start_time
            
            if elapsed >= self.stop_duration:
                # Temps écoulé (30 secondes), retourner au centre puis recommencer
                if self.capture_index == 3:
                    if self.ptz_step == 'idle':
                        if self.use_presets:
                            self.call_ptz_preset(self.get_preset_id('centre'))
                        else:
                            self.set_ptz_home()
                        self.ptz_wait_start = time.time()
                        self.ptz_step = 'waiting_home_final'
                    elif self.ptz_step == 'waiting_home_final' and self.state == 'STOPPED':
                        self.get_logger().info(f"Séquence #{self.sequence_count} terminée. Caméra revenue au centre.")
                        self.get_logger().info("Réinitialisation pour nouvelle séquence...")
                        self.state = 'RESETTING'
                        self.reset_start_time = time.time()
                else:
                    self.state = 'RESETTING'
                    self.reset_start_time = time.time()
            elif self.capture_index == 0:
                # Première capture : 90° à gauche
                if elapsed >= 0.0 and self.ptz_step == 'idle':
                    # À 0 secondes : appeler le preset gauche
                    if self.use_presets:
                        preset_id = self.get_preset_id('gauche')
                        self.get_logger().info(f"Positionnement PTZ: {self.get_preset_name(preset_id)}...")
                        self.call_ptz_preset(preset_id)
                    else:
                        self.get_logger().info("Positionnement PTZ: 90° à gauche...")
                        self.start_ptz_movement(-1.0, 0.0, 5.0)
                    self.ptz_step = 'moving_left'
                elif elapsed >= 5.0 and self.ptz_step == 'moving_left' and self.state == 'STOPPED':
                    # À 5 secondes : déclencher la capture
                    self.trigger_capture(self.get_preset_name(self.get_preset_id('gauche')))
                    self.capture_index = 1
                    self.ptz_step = 'idle'
            elif self.capture_index == 1:
                # Deuxième capture : vers le haut
                if elapsed >= 10.0:
                    if self.ptz_step == 'idle':
                        # À 10 secondes : retour au centre
                        if self.use_presets:
                            preset_id = self.get_preset_id('centre')
                            self.get_logger().info(f"Retour au {self.get_preset_name(preset_id)}...")
                            self.call_ptz_preset(preset_id)
                        else:
                            self.get_logger().info("Retour au centre...")
                            self.set_ptz_home()
                            self.ptz_wait_start = time.time()
                        self.ptz_step = 'waiting_center_h'
                    elif self.ptz_step == 'waiting_center_h' and self.state == 'STOPPED':
                        # Après stabilisation, aller vers le haut
                        if self.use_presets:
                            preset_id = self.get_preset_id('haut')
                            self.get_logger().info(f"Positionnement PTZ: {self.get_preset_name(preset_id)}...")
                            self.call_ptz_preset(preset_id)
                        else:
                            self.get_logger().info("Positionnement PTZ: vers le haut...")
                            self.start_ptz_movement(0.0, 1.0, 2.5)
                        self.ptz_step = 'moving_up'
                elif elapsed >= 15.0 and self.ptz_step == 'moving_up' and self.state == 'STOPPED':
                    # À 15 secondes : déclencher la capture
                    self.trigger_capture(self.get_preset_name(self.get_preset_id('haut')))
                    self.capture_index = 2
                    self.ptz_step = 'idle'
            elif self.capture_index == 2:
                # Troisième capture : 90° à droite
                if elapsed >= 20.0:
                    if self.ptz_step == 'idle':
                        # À 20 secondes : retour au centre
                        if self.use_presets:
                            preset_id = self.get_preset_id('centre')
                            self.get_logger().info(f"Retour au {self.get_preset_name(preset_id)}...")
                            self.call_ptz_preset(preset_id)
                        else:
                            self.get_logger().info("Retour au centre...")
                            self.set_ptz_home()
                            self.ptz_wait_start = time.time()
                        self.ptz_step = 'waiting_center_v'
                    elif self.ptz_step == 'waiting_center_v' and self.state == 'STOPPED':
                        # Après stabilisation, aller à droite
                        if self.use_presets:
                            preset_id = self.get_preset_id('droite')
                            self.get_logger().info(f"Positionnement PTZ: {self.get_preset_name(preset_id)}...")
                            self.call_ptz_preset(preset_id)
                        else:
                            self.get_logger().info("Positionnement PTZ: 90° à droite...")
                            self.start_ptz_movement(1.0, 0.0, 5.0)
                        self.ptz_step = 'moving_right'
                elif elapsed >= 25.0 and self.ptz_step == 'moving_right' and self.state == 'STOPPED':
                    # À 25 secondes : déclencher la capture
                    self.trigger_capture(self.get_preset_name(self.get_preset_id('droite')))
                    self.capture_index = 3
                    self.get_logger().info("Toutes les captures effectuées.")
                    self.ptz_step = 'idle'
        
        elif self.state == 'RESETTING':
            # Attendre un peu avant de recommencer
            if time.time() - self.reset_start_time >= 2.0:
                self.reset_sequence()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSequence()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt de la séquence...")
        node.stop_robot()
        node.set_ptz_home()
    except Exception as e:
        node.get_logger().error(f"Erreur dans la séquence: {e}")
    finally:
        try:
            if rclpy.ok():
                node.stop_robot()
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
