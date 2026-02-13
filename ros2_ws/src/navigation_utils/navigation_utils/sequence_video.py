#!/usr/bin/env python3
"""
Script d'automatisation pour faire avancer le robot en continu tout en enregistrant 
UNE SEULE vidéo avec balayage PTZ en quart de sphère devant la caméra.

Le robot avance à vitesse minimale pendant que la caméra PTZ effectue un balayage
continu en quart de sphère (pan de gauche à droite, tilt de bas en haut).

Ce script utilise les nœuds ROS2 existants :
- ptz_controller : via /ptz/cmd_vel pour les mouvements PTZ
- ffmpeg : pour enregistrer les vidéos depuis le flux RTSP
- odometry/filtered : pour mesurer la distance parcourue

Pattern de balayage quart de sphère :
- Balayage continu répétitif qui couvre un quart de sphère devant la caméra
  - Pan : de gauche (-90°) vers droite (0°) ou de gauche (-1.0) vers centre (0.0)
  - Tilt : de bas (0°) vers haut (90°) ou de bas (0.0) vers haut (1.0)
  - Le balayage se répète en boucle pendant toute la durée de la vidéo
  - Le robot avance à vitesse minimale pendant tout l'enregistrement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import time
import subprocess
import os
import signal
from pathlib import Path


class RobotSequenceVideo(Node):
    """Nœud ROS2 pour automatiser une séquence de mouvement et enregistrements vidéo PTZ en boucle"""
    
    def __init__(self):
        super().__init__('robot_sequence_video')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ptz_cmd_vel_pub = self.create_publisher(Twist, '/ptz/cmd_vel', 10)
        self.ptz_preset_pub = self.create_publisher(Int32, '/ptz/preset', 10)
        
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
        self.state = 'WAITING_START'  # WAITING_START, RECORDING
        
        # Paramètres
        self.declare_parameter('robot_speed', 0.05)  # Vitesse minimale de déplacement (m/s) - très lent
        
        # Configuration RTSP et vidéo
        self.declare_parameter('rtsp_url', 'rtsp://admin:admin@192.168.5.163:554/live/av0')
        self.declare_parameter('video_output_dir', str(Path.home() / 'Documents' / 'ANDRA_2025-2026' / 'video' / 'video_output'))
        self.declare_parameter('enable_video_adjustment', False)  # Activer retouches vidéo
        self.declare_parameter('video_brightness', 1.0)
        self.declare_parameter('video_contrast', 1.0)
        self.declare_parameter('video_gamma', 1.0)
        
        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.video_output_dir = self.get_parameter('video_output_dir').get_parameter_value().string_value
        self.enable_video_adjustment = self.get_parameter('enable_video_adjustment').get_parameter_value().bool_value
        self.video_brightness = self.get_parameter('video_brightness').get_parameter_value().double_value
        self.video_contrast = self.get_parameter('video_contrast').get_parameter_value().double_value
        self.video_gamma = self.get_parameter('video_gamma').get_parameter_value().double_value
        
        # Créer le répertoire de sortie
        os.makedirs(self.video_output_dir, exist_ok=True)
        
        # Configuration PTZ - Balayage quart de sphère
        # Balayage continu répétitif qui couvre un quart de sphère devant la caméra
        self.declare_parameter('ptz_sweep_duration', 30.0)  # Durée d'un cycle de balayage complet (secondes)
        self.declare_parameter('ptz_start_pan', -1.0)  # Position pan de départ (gauche = -1.0, centre = 0.0)
        self.declare_parameter('ptz_end_pan', 0.0)  # Position pan d'arrivée (centre = 0.0)
        self.declare_parameter('ptz_start_tilt', 0.0)  # Position tilt de départ (bas = 0.0)
        self.declare_parameter('ptz_end_tilt', 1.0)  # Position tilt d'arrivée (haut = 1.0)
        self.declare_parameter('ptz_speed_factor', 10.0)  # Facteur multiplicateur pour les vitesses PTZ (augmenté pour mouvement visible)
        
        self.ptz_sweep_duration = self.get_parameter('ptz_sweep_duration').get_parameter_value().double_value
        self.ptz_start_pan = self.get_parameter('ptz_start_pan').get_parameter_value().double_value
        self.ptz_start_tilt = self.get_parameter('ptz_start_tilt').get_parameter_value().double_value
        self.ptz_end_pan = self.get_parameter('ptz_end_pan').get_parameter_value().double_value
        self.ptz_end_tilt = self.get_parameter('ptz_end_tilt').get_parameter_value().double_value
        self.ptz_speed_factor = self.get_parameter('ptz_speed_factor').get_parameter_value().double_value
        
        # Timer pour la séquence
        self.timer = self.create_timer(0.1, self.sequence_loop)  # 10 Hz
        
        self.get_logger().info("=== Enregistrement vidéo CONTINU avec balayage quart de sphère ===")
        self.get_logger().info(f"Vitesse robot: {self.robot_speed} m/s")
        self.get_logger().info(f"Durée cycle balayage PTZ: {self.ptz_sweep_duration}s")
        self.get_logger().info(f"Balayage: Pan [{self.ptz_start_pan:.1f} → {self.ptz_end_pan:.1f}], Tilt [{self.ptz_start_tilt:.1f} → {self.ptz_end_tilt:.1f}]")
        self.get_logger().info(f"Facteur vitesse PTZ: {self.ptz_speed_factor}x")
        self.get_logger().info(f"Sortie vidéo: {self.video_output_dir}")
        self.get_logger().info("UNE SEULE vidéo continue jusqu'à l'arrêt manuel (Ctrl+C)")
        self.get_logger().info("Le robot avance à vitesse minimale pendant l'enregistrement")
        
        # Temps de démarrage et gestion des états
        self.record_start_time = None
        self.ffmpeg_process = None
        self.video_filename = None
        
        # État du balayage PTZ (balayage répétitif)
        self.ptz_sweep_start_time = None
        self.ptz_sweep_progress = 0.0  # 0.0 à 1.0 pour chaque cycle
        self.ptz_sweep_cycle = 0  # Numéro du cycle de balayage
        self.ptz_positioning_done = False
        
    def odom_callback(self, msg):
        """Callback pour recevoir l'odométrie"""
        self.current_position = msg.pose.pose.position
        
        # Initialiser et démarrer l'enregistrement au premier message
        if self.state == 'WAITING_START':
            self.state = 'RECORDING'
            # Activer l'auto-exposure avant de commencer
            self.set_auto_exposure()
            # Démarrer l'enregistrement vidéo (sans limite de durée)
            if not self.start_video_recording():
                self.get_logger().error("Échec du démarrage de l'enregistrement")
                return
            self.get_logger().info("Enregistrement vidéo démarré - Appuyez sur Ctrl+C pour arrêter")
    
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
    
    def move_ptz(self, pan_speed, tilt_speed):
        """Publie une commande de vitesse pour la PTZ via /ptz/cmd_vel"""
        twist = Twist()
        twist.angular.z = float(pan_speed)  # Pan (rotation horizontale)
        twist.linear.y = float(tilt_speed)  # Tilt (rotation verticale)
        self.ptz_cmd_vel_pub.publish(twist)
        # Log périodique pour déboguer
        if hasattr(self, '_last_ptz_log_time'):
            if time.time() - self._last_ptz_log_time > 2.0:  # Log toutes les 2 secondes
                self.get_logger().debug(f"PTZ cmd_vel: pan={pan_speed:.3f}, tilt={tilt_speed:.3f}")
                self._last_ptz_log_time = time.time()
        else:
            self._last_ptz_log_time = time.time()
    
    def stop_ptz(self):
        """Arrête le mouvement de la PTZ"""
        self.move_ptz(0.0, 0.0)
    
    def reset_ptz_sweep(self):
        """Réinitialise le balayage PTZ pour un nouveau cycle"""
        self.ptz_sweep_progress = 0.0
        self.ptz_sweep_cycle += 1
        self.ptz_sweep_start_time = time.time()
        self.get_logger().debug(f"Cycle de balayage #{self.ptz_sweep_cycle} démarré")
    
    def set_auto_exposure(self):
        """Active l'auto-exposure matériel via VISCA"""
        try:
            import socket
            cam_ip = '192.168.5.163'
            cam_port = 1259
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(5.0)
                sock.connect((cam_ip, cam_port))
                # Commande VISCA Full Auto: 81 01 04 39 00 FF
                command = bytes([0x81, 0x01, 0x04, 0x39, 0x00, 0xFF])
                sock.sendall(command)
            self.get_logger().info("Auto-exposure matériel activé")
            return True
        except Exception as e:
            self.get_logger().warn(f"Impossible d'activer l'auto-exposure: {e}")
            return False
    
    def start_video_recording(self):
        """Démarre l'enregistrement vidéo avec ffmpeg (sans limite de durée)"""
        if self.ffmpeg_process is not None:
            self.get_logger().warn("Enregistrement déjà en cours")
            return False
        
        # Générer le nom de fichier avec timestamp
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.video_filename = os.path.join(
            self.video_output_dir,
            f"video_continuous_{timestamp}.mp4"
        )
        
        # Construire la commande ffmpeg (sans limite de durée - s'arrête avec Ctrl+C)
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',  # Écraser si existe
            '-rtsp_transport', 'tcp',
            '-i', self.rtsp_url,
            # Pas de '-t' pour permettre l'enregistrement continu jusqu'à l'arrêt manuel
        ]
        
        # Ajouter les filtres vidéo si activés
        if self.enable_video_adjustment:
            brightness_ffmpeg = max(-1.0, min(1.0, (self.video_brightness - 1.0) / 2.0))
            ffmpeg_cmd.extend([
                '-vf', f'eq=brightness={brightness_ffmpeg}:contrast={self.video_contrast}:gamma={self.video_gamma}',
                '-c:v', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
            ])
        else:
            ffmpeg_cmd.extend(['-c', 'copy'])  # Copie directe sans réencodage
        
        ffmpeg_cmd.append(self.video_filename)
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.record_start_time = time.time()
            self.get_logger().info(f"Démarrage enregistrement vidéo: {self.video_filename}")
            return True
        except FileNotFoundError:
            self.get_logger().error("FFmpeg non trouvé. Installez : sudo apt install ffmpeg")
            return False
        except Exception as e:
            self.get_logger().error(f"Erreur lors du démarrage de l'enregistrement: {e}")
            return False
    
    def stop_video_recording(self):
        """Arrête l'enregistrement vidéo"""
        if self.ffmpeg_process is None:
            return
        
        try:
            # Envoyer SIGINT pour terminer proprement ffmpeg
            self.ffmpeg_process.send_signal(signal.SIGINT)
            self.ffmpeg_process.wait(timeout=5)
            self.get_logger().info(f"Enregistrement vidéo terminé: {self.video_filename}")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timeout lors de l'arrêt de ffmpeg, kill forcé")
            self.ffmpeg_process.kill()
            self.ffmpeg_process.wait()
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'arrêt de l'enregistrement: {e}")
            if self.ffmpeg_process.poll() is None:
                self.ffmpeg_process.kill()
        
        self.ffmpeg_process = None
        self.video_filename = None
    
    def update_ptz_sweep(self):
        """
        Pattern Scanner de Voûte : Balayage horizontal (Pan) de gauche à droite.
        Le Tilt reste fixé vers le haut pour inspecter la voûte.
        """
        import math
        
        if self.ptz_sweep_start_time is None:
            self.reset_ptz_sweep()
            return
        
        elapsed = time.time() - self.ptz_sweep_start_time
        
        # On utilise une fonction Sinus pour un mouvement de va-et-vient fluide
        # ptz_sweep_duration est le temps pour un aller-retour complet
        frequence = (2 * math.pi) / self.ptz_sweep_duration
        
        # 1. Calcul du PAN (Balayage latéral)
        # On veut que le pan oscille entre ptz_start_pan et ptz_end_pan
        # Position théorique pour le log (optionnel)
        amplitude_pan = (self.ptz_end_pan - self.ptz_start_pan) / 2
        milieu_pan = (self.ptz_end_pan + self.ptz_start_pan) / 2
        
        # Calcul de la vitesse du Pan (Dérivée du mouvement sinusoïdal)
        # v = A * w * cos(w*t)
        pan_speed = amplitude_pan * frequence * math.cos(frequence * elapsed)
        
        # 2. Calcul du TILT (Fixe vers la voûte)
        # On se positionne à la valeur maximale définie (ex: 1.0 pour 90°)
        # On applique une petite vitesse corrective si la caméra dévie (ou vitesse 0)
        current_tilt_target = self.ptz_end_tilt
        
        # Si on est au début du cycle, on s'assure que le tilt est bien positionné
        tilt_speed = 0.0
        if self.ptz_sweep_progress < 0.1: # Au début de chaque cycle
            tilt_speed = 0.1 # Légère poussée pour maintenir le plafond
            
        # Application du facteur de vitesse pour ROS
        final_pan_speed = pan_speed * self.ptz_speed_factor
        final_tilt_speed = tilt_speed * self.ptz_speed_factor

        # Sécurité limites
        final_pan_speed = max(-1.0, min(1.0, final_pan_speed))
        
        self.move_ptz(final_pan_speed, final_tilt_speed)
        
        # Mise à jour du progrès pour le cycle
        self.ptz_sweep_progress = (elapsed % self.ptz_sweep_duration) / self.ptz_sweep_duration
    
    def sequence_loop(self):
        """Boucle principale - mouvement continu avec balayage quart de sphère"""
        if self.state == 'WAITING_START':
            return
        
        elif self.state == 'RECORDING':
            if self.current_position is None:
                return
            
            # Le robot avance EN CONTINU à vitesse minimale
            self.move_robot(self.robot_speed)
            
            # Positionner la PTZ au point de départ si pas encore fait
            if not self.ptz_positioning_done:
                self.get_logger().info("Positionnement PTZ au point de départ...")
                # Aller rapidement à la position de départ avec une vitesse élevée
                # Calculer la vitesse nécessaire pour aller rapidement à la position de départ
                # On utilise une vitesse fixe pour le positionnement initial
                pan_to_start = self.ptz_start_pan - 0.0  # Depuis le centre (0.0)
                tilt_to_start = self.ptz_start_tilt - 0.0  # Depuis le centre (0.0)
                # Vitesse rapide pour le positionnement initial (0.5 unités/seconde)
                pan_speed_init = 0.5 if pan_to_start > 0 else -0.5 if pan_to_start < 0 else 0.0
                tilt_speed_init = 0.5 if tilt_to_start > 0 else -0.5 if tilt_to_start < 0 else 0.0
                self.move_ptz(pan_speed_init, tilt_speed_init)
                
                # Attendre un court instant pour le positionnement
                if self.ptz_sweep_start_time is None:
                    self.ptz_sweep_start_time = time.time()
                    self.positioning_start_time = time.time()
                
                # Après 2 secondes, arrêter le positionnement et commencer le balayage
                if time.time() - self.positioning_start_time >= 2.0:
                    self.stop_ptz()
                    self.ptz_positioning_done = True
                    self.ptz_sweep_start_time = time.time()  # Réinitialiser pour le balayage
                    self.get_logger().info("PTZ positionnée, début du balayage quart de sphère...")
            
            # Mettre à jour le balayage PTZ en continu (répétitif)
            if self.ptz_positioning_done:
                self.update_ptz_sweep()
            
            # Vérifier si l'enregistrement est toujours actif
            if self.ffmpeg_process is not None and self.ffmpeg_process.poll() is not None:
                # FFmpeg s'est arrêté (ne devrait pas arriver normalement)
                self.get_logger().warn("FFmpeg s'est arrêté de manière inattendue")
                self.stop_video_recording()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSequenceVideo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt de la séquence...")
        node.stop_robot()
        node.stop_ptz()
        node.stop_video_recording()
    except Exception as e:
        node.get_logger().error(f"Erreur dans la séquence: {e}")
    finally:
        try:
            if rclpy.ok():
                node.stop_robot()
                node.stop_ptz()
                node.stop_video_recording()
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
