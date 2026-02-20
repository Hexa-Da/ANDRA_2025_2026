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
        self.shutdown_requested = False

        # Paramètres
        self.target_distance = 1.0  # 1 mètre
        self.stop_duration = 30.0  # 30 secondes
        self.robot_speed = 0.3  # m/s (vitesse de déplacement)
        
        # Configuration PTZ - Séquence précise avec durées fixes
        # Séquence : gauche(3.6s) → stab(0.5s) → capture → attendre(0.5s)
        #          → haut(3s) → stab(0.5s) → capture → attendre(0.5s)
        #          → haut(4s) → stab(0.5s) → capture → attendre(0.5s) → Home(4.5s)
        #          → droite(3.6s) → stab(0.5s) → capture → attendre(0.5s)
        #          → haut(3s) → stab(0.5s) → capture → attendre(0.5s)
        #          → Home(3s)
        self.declare_parameter('ptz_move_90_duration', 3.6)   # Durée mouvement pan (gauche/droite)
        self.declare_parameter('ptz_move_up_short_duration', 3.0)   # Durée mouvement tilt (haut) court
        self.declare_parameter('ptz_move_up_long_duration', 4.0)   # Durée mouvement tilt (haut) long
        self.declare_parameter('ptz_stable_duration', 0.5)    # Durée stabilisation avant capture
        self.declare_parameter('delay_after_capture', 0.5)     # Attente après capture avant mouvement suivant
        self.declare_parameter('ptz_home_duration_mid', 4.5)   # Durée retour Home après capture 3
        self.declare_parameter('ptz_home_duration_final', 1.5)   # Durée retour Home final
        
        self.ptz_move_90_duration = self.get_parameter('ptz_move_90_duration').get_parameter_value().double_value
        self.ptz_move_up_short_duration = self.get_parameter('ptz_move_up_short_duration').get_parameter_value().double_value
        self.ptz_move_up_long_duration = self.get_parameter('ptz_move_up_long_duration').get_parameter_value().double_value
        self.ptz_stable_duration = self.get_parameter('ptz_stable_duration').get_parameter_value().double_value
        self.delay_after_capture = self.get_parameter('delay_after_capture').get_parameter_value().double_value
        self.ptz_home_duration_mid = self.get_parameter('ptz_home_duration_mid').get_parameter_value().double_value
        self.ptz_home_duration_final = self.get_parameter('ptz_home_duration_final').get_parameter_value().double_value
        
        # Timer pour la séquence
        self.timer = self.create_timer(0.1, self.sequence_loop)  # 10 Hz
        
        self.get_logger().info("=== Séquence robot démarrée (mode boucle) ===")
        self.get_logger().info(f"Distance cible: {self.target_distance}m")
        self.get_logger().info(f"Durée d'arrêt: {self.stop_duration}s")
        self.get_logger().info(
            f"5 captures PTZ: Gauche, Haut(3s), Haut(3.5s)+Home, Droite, Haut(3s), puis Home final"
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
        self.step_start_time = None  # Timestamp du début de l'étape actuelle
        self.robot_can_move = False  # Flag pour permettre au robot de repartir pendant le Home final
        
    def reset_sequence(self):
        """Réinitialise la séquence pour une nouvelle itération"""
        self.initial_position = None
        self.current_position = None
        self.state = 'WAITING_START'
        self.capture_index = 0
        self.ptz_step = 'idle'
        self.sequence_count += 1
        self.robot_can_move = False
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
        """Publie une commande de vitesse avec sécurité contextuelle"""
        if self.shutdown_requested:
            return
        try:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(twist)
        except Exception:
            pass
    
    def stop_robot(self):
        """Arrête le robot"""
        self.move_robot(0.0, 0.0)
    
    def move_ptz(self, pan, tilt):
        if self.shutdown_requested:
            return
        try:
            twist = Twist()
            twist.angular.z = float(pan)
            twist.linear.y = float(tilt)
            self.ptz_cmd_vel_pub.publish(twist)
        except Exception:
            pass
    
    def stop_ptz(self):
        """Arrête le mouvement de la PTZ immédiatement
        
        Les mouvements sont contrôlés par durée. Quand la durée est atteinte,
        stop_ptz() est appelé pour arrêter la caméra à sa position actuelle.
        """
        self.move_ptz(0.0, 0.0)
    
    def stop_then_home_ptz(self):
        """Arrête tout et force le Home via socket (interruption)"""

        self.shutdown_requested = True
        print("\nArrêt PTZ et retour Home matériel...\n\n")

        try:
            import socket
            cam_ip = '192.168.5.163'
            cam_port = 1259 
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                sock.connect((cam_ip, cam_port))
                
                # A. STOP (Crucial : annule les cmd_vel en cours dans le buffer caméra)
                sock.sendall(bytes([0x81, 0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03, 0xFF]))
                time.sleep(0.1)
                
                # B. HOME
                sock.sendall(bytes([0x81, 0x01, 0x06, 0x04, 0xFF]))
        except Exception as e:
            print(f"Erreur Socket: {e}")
    
    def start_ptz_movement(self, pan, tilt, duration):
        """Démarre un mouvement PTZ pendant une durée donnée
        
        La durée détermine quand arrêter le mouvement pour atteindre la position désirée.
        Ajustez les durées dans les paramètres ROS pour calibrer les positions.
        """
        self.ptz_move_start_time = time.time()
        self.ptz_move_duration = duration
        self.move_ptz(pan, tilt)
        self.state = 'PTZ_MOVING'
        direction = []
        if abs(pan) > 0.1:
            direction.append("gauche" if pan < 0 else "droite")
        if abs(tilt) > 0.1:
            direction.append("bas" if tilt < 0 else "haut")
        if not direction:
            direction.append("centre")
        self.get_logger().info(f"PTZ: Mouvement vers {'-'.join(direction)} pendant {duration}s")
    
    def return_ptz_to_center(self, duration=None, allow_robot_move=False):
        self.stop_ptz()
        time.sleep(0.1)
        if not self.shutdown_requested:
            msg = Int32()
            msg.data = -1
            self.ptz_preset_pub.publish(msg)
        self.state = 'WAITING_STABLE'
        self.stable_start_time = time.time()
        self.center_wait_duration = duration if duration is not None else 5.0
        self.robot_can_move = allow_robot_move
    
    def trigger_capture(self, description=""):
        """Déclenche une capture via le topic /trigger_capture (std_msgs/String)"""
        msg = String()
        msg.data = description if description else ""
        self.trigger_capture_pub.publish(msg)
        if description:
            self.get_logger().info(f"Capture déclenchée: {description}")
    
    def sequence_loop(self):
        """Boucle principale de la séquence"""
        if self.state == 'WAITING_START' or self.shutdown_requested:
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
        
        elif self.state == 'RESETTING':
            # Si le robot peut repartir pendant le Home final, démarrer immédiatement
            if self.robot_can_move and self.capture_index == 6:
                # Le robot peut repartir même si la PTZ n'a pas fini de revenir au centre
                self.get_logger().info("Robot repart pendant le retour au centre de la PTZ...")
                self.reset_sequence()
                return
            
            # Attendre un peu avant de recommencer (sauf si robot peut déjà bouger)
            if time.time() - self.reset_start_time >= 2.0:
                self.reset_sequence()
        
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
            # Attendre la stabilisation après le mouvement PTZ ou le retour au centre
            wait_duration = getattr(self, 'center_wait_duration', None)
            if wait_duration is not None:
                # Retour au centre en cours
                elapsed = time.time() - self.stable_start_time
                if elapsed >= wait_duration:
                    self.state = 'STOPPED'
                    self.center_wait_duration = None
                    self.robot_can_move = False
                elif self.robot_can_move:
                    # Si le robot peut repartir, permettre le mouvement même pendant le Home
                    # Le robot peut commencer à bouger pendant que la PTZ revient au centre
                    pass
            else:
                # Stabilisation normale après mouvement
                if time.time() - self.stable_start_time >= self.ptz_stable_duration:
                    self.state = 'STOPPED'
        
        elif self.state == 'STOPPED':
            elapsed = time.time() - self.stop_start_time
            
            # Si le robot peut repartir pendant le Home final, permettre le mouvement
            if self.robot_can_move and self.capture_index == 6:
                # Le robot peut repartir même si la PTZ n'a pas fini de revenir au centre
                if elapsed >= self.stop_duration:
                    self.get_logger().info("Robot repart pendant le retour au centre de la PTZ...")
                    self.state = 'RESETTING'
                    self.reset_start_time = time.time()
                    return
            
            if elapsed >= self.stop_duration:
                # Temps écoulé (30 secondes), retourner au centre puis recommencer
                if self.capture_index == 6:
                    # Séquence complètement terminée
                    if not self.robot_can_move:
                        # Si le robot ne peut pas repartir pendant le Home, attendre que le Home soit terminé
                        self.get_logger().info(f"Séquence #{self.sequence_count} terminée.")
                        self.get_logger().info("Réinitialisation pour nouvelle séquence...")
                        self.state = 'RESETTING'
                        self.reset_start_time = time.time()
                    # Si robot_can_move, la logique ci-dessus gère déjà le cas
                else:
                    # Pour les autres captures, passer à RESETTING normalement
                    self.state = 'RESETTING'
                    self.reset_start_time = time.time()
            elif self.capture_index == 0:
                # Séquence gauche : mouvement(3.5s) → stab(0.5s) → capture → attendre(0.5s)
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
                    # Après capture + attendre(0.5s) : passer à la position suivante (haut)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.ptz_step = 'idle'
                        self.capture_index = 1
            elif self.capture_index == 1:
                # Séquence haut #1 : mouvement(3s) → stab(0.5s) → capture → attendre(0.5s)
                if self.ptz_step == 'idle':
                    # Aller vers le haut depuis la position gauche (3s)
                    self.get_logger().info("Positionnement PTZ: vers le haut (1/2)...")
                    self.start_ptz_movement(0.0, 1.0, self.ptz_move_up_short_duration)
                    self.ptz_step = 'moving_up'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_up' and self.state == 'STOPPED':
                    # Après mouvement(3s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_up_short_duration + self.ptz_stable_duration):
                        self.trigger_capture("Haut-1")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : passer à la position suivante (haut #2)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.ptz_step = 'idle'
                        self.capture_index = 2
            elif self.capture_index == 2:
                # Séquence haut #2 : mouvement(3.5s) → stab(0.5s) → capture → attendre(0.5s) → Home(6s)
                if self.ptz_step == 'idle':
                    # Continuer vers le haut (3.5s)
                    self.get_logger().info("Positionnement PTZ: vers le haut (2/2)...")
                    self.start_ptz_movement(0.0, 1.0, self.ptz_move_up_long_duration)
                    self.ptz_step = 'moving_up'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_up' and self.state == 'STOPPED':
                    # Après mouvement(3.5s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_up_long_duration + self.ptz_stable_duration):
                        self.trigger_capture("Haut-2")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : retour Home(6s)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.return_ptz_to_center(self.ptz_home_duration_mid, allow_robot_move=False)
                        self.ptz_step = 'waiting_home_mid'
                        self.capture_index = 3
            elif self.capture_index == 3:
                # Attendre que le Home soit terminé avant de passer à droite
                if self.ptz_step == 'waiting_home_mid' and self.state == 'STOPPED':
                    # Home terminé, passer à la position suivante (droite)
                    self.ptz_step = 'idle'
                    self.capture_index = 4
            elif self.capture_index == 4:
                # Séquence droite : mouvement(3.5s) → stab(0.5s) → capture → attendre(0.5s)
                if self.ptz_step == 'idle':
                    # Aller vers la droite (3.5s)
                    self.get_logger().info("Positionnement PTZ: 90° à droite...")
                    self.start_ptz_movement(1.0, 0.0, self.ptz_move_90_duration)
                    self.ptz_step = 'moving_right'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_right' and self.state == 'STOPPED':
                    # Après mouvement(3.5s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_90_duration + self.ptz_stable_duration):
                        self.trigger_capture("Droite")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : passer à la position suivante (haut)
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.ptz_step = 'idle'
                        self.capture_index = 5
            elif self.capture_index == 5:
                # Séquence haut #3 : mouvement(3s) → stab(0.5s) → capture → attendre(0.5s)
                if self.ptz_step == 'idle':
                    # Aller vers le haut depuis la position droite (3s)
                    self.get_logger().info("Positionnement PTZ: vers le haut (final)...")
                    self.start_ptz_movement(0.0, 1.0, self.ptz_move_up_short_duration)
                    self.ptz_step = 'moving_up'
                    self.step_start_time = time.time()
                elif self.ptz_step == 'moving_up' and self.state == 'STOPPED':
                    # Après mouvement(3s) + stab(0.5s) : capture
                    if time.time() - self.step_start_time >= (self.ptz_move_up_short_duration + self.ptz_stable_duration):
                        self.trigger_capture("Haut-3")
                        self.get_logger().info("Toutes les captures effectuées.")
                        self.ptz_step = 'waiting_after_capture'
                        self.step_start_time = time.time()
                elif self.ptz_step == 'waiting_after_capture':
                    # Après capture + attendre(0.5s) : retour Home(5s) - robot peut repartir
                    if time.time() - self.step_start_time >= self.delay_after_capture:
                        self.return_ptz_to_center(self.ptz_home_duration_final, allow_robot_move=True)
                        self.ptz_step = 'waiting_home_final'
                        self.capture_index = 6


def main(args=None):
    rclpy.init(args=args)
    node = RobotSequence()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterruption détectée...")
    finally:
        # ÉTAPE 1: On bloque toutes les futures publications ROS
        node.shutdown_requested = True
        
        # ÉTAPE 2: On tente l'arrêt du robot avant que le contexte ne meure totalement
        # On utilise un try/except car le contexte ROS est déjà fragile ici
        try:
            print("[CLEANUP] Arrêt du robot...")
            node.stop_robot()
        except Exception:
            print("[CLEANUP] Note: ROS2 n'a pas pu envoyer le stop_robot (contexte expiré).")
        
        # ÉTAPE 3: On exécute le HOME via Socket (indépendant de ROS2)
        node.stop_then_home_ptz()
        
        # ÉTAPE 4: Fermeture propre
        time.sleep(0.2) # Laisser le temps aux derniers paquets de sortir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
