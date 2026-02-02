#!/usr/bin/env python3
"""
Script d'automatisation pour faire avancer le robot de 1m, 
arrêter pendant 30 secondes avec captures PTZ, puis repartir en boucle.

Ce script utilise les nœuds ROS2 existants :
- ptz_controller : via /ptz/cmd_vel pour les mouvements PTZ
- image_publisher : via /trigger_capture pour déclencher les captures
- odometry/filtered : pour mesurer la distance parcourue
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import time


class RobotSequence(Node):
    """Nœud ROS2 pour automatiser une séquence de mouvement et captures PTZ en boucle"""
    
    def __init__(self):
        super().__init__('robot_sequence')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ptz_cmd_vel_pub = self.create_publisher(Twist, '/ptz/cmd_vel', 10)
        self.ptz_preset_pub = self.create_publisher(Int32, '/ptz/preset', 10)
        self.trigger_capture_pub = self.create_publisher(String, '/trigger_capture', 10)
        
        # Subscriber pour l'odométrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # État du robot
        self.initial_position = None
        self.current_position = None
        self.state = 'WAITING_START'  # WAITING_START, MOVING, STOPPED, PTZ_MOVING, WAITING_STABLE, RESETTING
        
        # Paramètres
        self.target_distance = 1.0  # 1 mètre
        self.stop_duration = 30.0  # 30 secondes
        self.robot_speed = 0.3  # m/s (vitesse de déplacement)
        
        # Configuration PTZ - Séquence précise avec durées fixes
        # Séquence : gauche(3.5s) → stab(0.5s) → capture → attendre(0.5s) → home(3.5s)
        #          → haut(5.5s) → stab(0.5s) → capture → attendre(0.5s) → home(5.5s)
        #          → droite(3.5s) → stab(0.5s) → capture → attendre(0.5s) → home(3.5s)
        self.declare_parameter('ptz_move_90_duration', 3.5)   # Durée mouvement pan (gauche/droite)
        self.declare_parameter('ptz_move_up_duration', 5.5)   # Durée mouvement tilt (haut)
        self.declare_parameter('ptz_stable_duration', 0.5)    # Durée stabilisation avant capture
        self.declare_parameter('delay_after_capture', 0.5)     # Attente après capture avant retour home
        self.declare_parameter('ptz_center_duration_pan', 3.5)  # Durée retour home depuis pan (gauche/droite)
        self.declare_parameter('ptz_center_duration_tilt', 5.5)  # Durée retour home depuis tilt (haut)
        
        self.ptz_move_90_duration = self.get_parameter('ptz_move_90_duration').get_parameter_value().double_value
        self.ptz_move_up_duration = self.get_parameter('ptz_move_up_duration').get_parameter_value().double_value
        self.ptz_stable_duration = self.get_parameter('ptz_stable_duration').get_parameter_value().double_value
        self.delay_after_capture = self.get_parameter('delay_after_capture').get_parameter_value().double_value
        self.ptz_center_duration_pan = self.get_parameter('ptz_center_duration_pan').get_parameter_value().double_value
        self.ptz_center_duration_tilt = self.get_parameter('ptz_center_duration_tilt').get_parameter_value().double_value   
        
        # Timer pour la séquence
        self.timer = self.create_timer(0.1, self.sequence_loop)  # 10 Hz
        
        self.get_logger().info("=== Séquence robot démarrée (mode boucle) ===")
        self.get_logger().info(f"Distance cible: {self.target_distance}m")
        self.get_logger().info(f"Durée d'arrêt: {self.stop_duration}s")
        self.get_logger().info(
            f"Captures à t={self.ptz_move_90_duration}s, {self.ptz_move_up_duration}s, {self.ptz_move_90_duration}s"
        )
        self.get_logger().info("PTZ: Utilisation de /ptz/cmd_vel (mouvements de base)")
        self.get_logger().info("Appuyez sur Ctrl+C pour arrêter la boucle")
        
        # Temps de démarrage et gestion des états
        self.start_time = None
        self.stop_start_time = None
        self.ptz_move_start_time = None
        self.capture_index = 0
        self.ptz_step = 'idle'
        self.sequence_count = 0
        self.center_wait_duration = None  # Durée spécifique pour le retour au centre
        self.last_movement_type = None  # 'pan' ou 'tilt' pour déterminer la durée de retour home
        self.step_start_time = None  # Timestamp du début de l'étape actuelle
        
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
        """Arrête le mouvement de la PTZ immédiatement
        
        Les mouvements sont contrôlés par durée. Quand la durée est atteinte,
        stop_ptz() est appelé pour arrêter la caméra à sa position actuelle.
        """
        self.move_ptz(0.0, 0.0)
    
    def start_ptz_movement(self, pan, tilt, duration):
        """Démarre un mouvement PTZ pendant une durée donnée
        
        La durée détermine quand arrêter le mouvement pour atteindre la position désirée.
        Ajustez les durées dans les paramètres ROS pour calibrer les positions.
        """
        self.ptz_move_start_time = time.time()
        self.ptz_move_duration = duration
        self.move_ptz(pan, tilt)
        self.state = 'PTZ_MOVING'
        # Enregistrer le type de mouvement pour déterminer la durée de retour home
        if abs(pan) > 0.1:
            self.last_movement_type = 'pan'
        elif abs(tilt) > 0.1:
            self.last_movement_type = 'tilt'
        direction = []
        if abs(pan) > 0.1:
            direction.append("gauche" if pan < 0 else "droite")
        if abs(tilt) > 0.1:
            direction.append("bas" if tilt < 0 else "haut")
        if not direction:
            direction.append("centre")
        self.get_logger().info(f"PTZ: Mouvement vers {'-'.join(direction)} pendant {duration}s")
    
    def return_ptz_to_center(self):
        """Retourne la PTZ au centre en utilisant le preset Home (-1)
        
        Utilise la durée appropriée selon le dernier mouvement (pan ou tilt)
        """
        # D'abord arrêter tout mouvement en cours
        self.stop_ptz()
        
        # Utiliser le preset Home (-1) pour revenir au centre
        msg = Int32()
        msg.data = -1  # -1 = Home (position centrale)
        self.ptz_preset_pub.publish(msg)
        
        # Passer en état d'attente avec la durée appropriée pour le retour au centre
        self.state = 'WAITING_STABLE'
        self.stable_start_time = time.time()
        # Utiliser la durée selon le type de mouvement précédent
        if self.last_movement_type == 'tilt':
            self.center_wait_duration = self.ptz_center_duration_tilt
            self.get_logger().info(f"PTZ: Commande Home (position centrale) - attente {self.ptz_center_duration_tilt}s")
        else:  # pan (gauche/droite)
            self.center_wait_duration = self.ptz_center_duration_pan
            self.get_logger().info(f"PTZ: Commande Home (position centrale) - attente {self.ptz_center_duration_pan}s")
    
    def trigger_capture(self, description=""):
        """Déclenche une capture via le topic /trigger_capture (std_msgs/String)"""
        msg = String()
        msg.data = description if description else ""
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
                    self.center_wait_duration = None  # Utiliser ptz_stable_duration pour la stabilisation
        
        elif self.state == 'WAITING_STABLE':
            # Attendre la stabilisation après le mouvement PTZ ou le retour au centre
            wait_duration = self.center_wait_duration if self.center_wait_duration is not None else self.ptz_stable_duration
            
            if time.time() - self.stable_start_time >= wait_duration:
                # Stabilisation terminée, retourner à STOPPED
                self.state = 'STOPPED'
                self.center_wait_duration = None  # Réinitialiser pour le prochain mouvement
        
        elif self.state == 'STOPPED':
            elapsed = time.time() - self.stop_start_time
            
            if elapsed >= self.stop_duration:
                # Temps écoulé (30 secondes), retourner au centre puis recommencer
                if self.capture_index == 3:
                    if self.ptz_step == 'idle':
                        # Retourner au centre
                        self.return_ptz_to_center()
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
                # Séquence gauche : mouvement(3.5s) → stab(0.5s) → capture → attendre(0.5s) → home(3.5s)
                if self.ptz_step == 'idle':
                    # Début : mouvement vers la gauche (3.5s)
                    self.get_logger().info("Positionnement PTZ: 90° à gauche...")
                    self.start_ptz_movement(-1.0, 0.0, self.ptz_move_90_duration)
                    self.ptz_step = 'moving_left'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_left' and self.state == 'STOPPED':
                    # Après mouvement(3.5s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_90_duration + self.ptz_stable_duration):
                        self.trigger_capture("Gauche")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : retour home (3.5s)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.return_ptz_to_center()
                        self.ptz_step = 'waiting_home'
                        self.capture_index = 1
            elif self.capture_index == 1:
                # Séquence haut : home terminé → mouvement(5s) → stab(0.5s) → capture → attendre(0.5s) → home(5s)
                if self.ptz_step == 'waiting_home' and self.state == 'STOPPED':
                    # Home terminé, aller vers le haut (5s)
                    self.get_logger().info("Positionnement PTZ: vers le haut...")
                    self.start_ptz_movement(0.0, 1.0, self.ptz_move_up_duration)
                    self.ptz_step = 'moving_up'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_up' and self.state == 'STOPPED':
                    # Après mouvement(5s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_up_duration + self.ptz_stable_duration):
                        self.trigger_capture("Haut")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : retour home (5s)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.return_ptz_to_center()
                        self.ptz_step = 'waiting_home'
                        self.capture_index = 2
            elif self.capture_index == 2:
                # Séquence droite : home terminé → mouvement(3.5s) → stab(0.5s) → capture → attendre(0.5s) → home(3.5s)
                if self.ptz_step == 'waiting_home' and self.state == 'STOPPED':
                    # Home terminé, aller vers la droite (3.5s)
                    self.get_logger().info("Positionnement PTZ: 90° à droite...")
                    self.start_ptz_movement(1.0, 0.0, self.ptz_move_90_duration)
                    self.ptz_step = 'moving_right'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_right' and self.state == 'STOPPED':
                    # Après mouvement(3.5s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_90_duration + self.ptz_stable_duration):
                        self.trigger_capture("Droite")
                        self.get_logger().info("Toutes les captures effectuées.")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : retour home (3.5s)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.return_ptz_to_center()
                        self.ptz_step = 'waiting_home_final'
                        self.capture_index = 3
        
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
        node.stop_ptz()
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
