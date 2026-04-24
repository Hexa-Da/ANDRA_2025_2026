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
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import subprocess
import os
import signal
import time as _time
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
        self.state = 'WAITING_START'
        self.shutdown_requested = False
        
        # Paramètres
        self.declare_parameter('robot_speed', 0.06)
        self.declare_parameter('enable_robot_motion', True)
        
        # Configuration RTSP et vidéo
        self.declare_parameter('rtsp_url', 'rtsp://admin:admin@192.168.5.163:554/live/av0')
        self.declare_parameter('video_output_dir', str(Path.home() / 'Documents' / 'ANDRA_2025-2026' / 'video' / 'video_output'))
        self.declare_parameter('enable_video_adjustment', False)
        self.declare_parameter('video_brightness', 1.0)
        self.declare_parameter('video_contrast', 1.0)
        self.declare_parameter('video_gamma', 1.0)
        
        self.robot_speed = self.get_parameter('robot_speed').get_parameter_value().double_value
        self.enable_robot_motion = self.get_parameter('enable_robot_motion').get_parameter_value().bool_value
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.video_output_dir = self.get_parameter('video_output_dir').get_parameter_value().string_value
        self.enable_video_adjustment = self.get_parameter('enable_video_adjustment').get_parameter_value().bool_value
        self.video_brightness = self.get_parameter('video_brightness').get_parameter_value().double_value
        self.video_contrast = self.get_parameter('video_contrast').get_parameter_value().double_value
        self.video_gamma = self.get_parameter('video_gamma').get_parameter_value().double_value
        
        os.makedirs(self.video_output_dir, exist_ok=True)
        
        # Configuration PTZ - Balayage quart de sphère
        self.declare_parameter('ptz_sweep_duration', 30.0)
        self.declare_parameter('ptz_start_pan', -1.0)
        self.declare_parameter('ptz_end_pan', 0.0)
        self.declare_parameter('ptz_start_tilt', 0.0)
        self.declare_parameter('ptz_end_tilt', 1.0)
        self.declare_parameter('ptz_speed_factor', 10.0)
        
        self.ptz_sweep_duration = self.get_parameter('ptz_sweep_duration').get_parameter_value().double_value
        self.ptz_start_pan = self.get_parameter('ptz_start_pan').get_parameter_value().double_value
        self.ptz_start_tilt = self.get_parameter('ptz_start_tilt').get_parameter_value().double_value
        self.ptz_end_pan = self.get_parameter('ptz_end_pan').get_parameter_value().double_value
        self.ptz_end_tilt = self.get_parameter('ptz_end_tilt').get_parameter_value().double_value
        self.ptz_speed_factor = self.get_parameter('ptz_speed_factor').get_parameter_value().double_value
        
        # Timer pour la séquence (50 Hz)
        self.timer = self.create_timer(0.02, self.sequence_loop)
        
        self.get_logger().info("=== Enregistrement vidéo continu avec balayage ===")
        self.get_logger().info(f"Vitesse robot: {self.robot_speed} m/s")
        self.get_logger().info(f"Durée cycle balayage PTZ: {self.ptz_sweep_duration}s")
        self.get_logger().info(f"Facteur vitesse PTZ: {self.ptz_speed_factor}x")
        self.get_logger().info(f"Sortie vidéo: {self.video_output_dir}")
        self.get_logger().info("Enregistrement de la vidéo en continu jusqu'à l'arrêt manuel (Ctrl+C)")
        
        # Timestamps ROS
        self.record_start_time: Time = None
        self.ffmpeg_process = None
        self.video_filename = None
        self.video_ready_flag_path = None
        
        # État du balayage PTZ
        self.ptz_sweep_start_time: Time = None
        self.ptz_sweep_progress = 0.0
        self.ptz_sweep_cycle = 0
        self.ptz_positioning_done = False
        self.positioning_start_time: Time = None
        self._last_ptz_log_time: Time = None

    def _now(self) -> Time:
        return self.get_clock().now()

    def _elapsed_sec(self, since: Time) -> float:
        """Retourne les secondes écoulées depuis un timestamp ROS."""
        if since is None:
            return 0.0
        return (self._now() - since).nanoseconds / 1e9

    def odom_callback(self, msg):
        """Callback pour recevoir l'odométrie"""
        self.current_position = msg.pose.pose.position
        
        if self.state == 'WAITING_START':
            self.state = 'RECORDING'
            self.set_auto_exposure()
            if not self.start_video_recording():
                self.get_logger().error("Échec du démarrage de l'enregistrement")
                return
    
    def calculate_distance(self, pos1, pos2):
        """Calcule la distance euclidienne entre deux positions"""
        return ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5
    
    def move_robot(self, linear_x, angular_z=0.0):
        """Publie une commande de vitesse pour le robot"""
        if not self.enable_robot_motion:
            return
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
        twist.angular.z = float(pan_speed)
        twist.linear.y = float(tilt_speed)
        self.ptz_cmd_vel_pub.publish(twist)
        if self._last_ptz_log_time is not None:
            if self._elapsed_sec(self._last_ptz_log_time) > 2.0:
                self.get_logger().debug(f"PTZ cmd_vel: pan={pan_speed:.3f}, tilt={tilt_speed:.3f}")
                self._last_ptz_log_time = self._now()
        else:
            self._last_ptz_log_time = self._now()
    
    def stop_ptz(self):
        """Arrête le mouvement de la PTZ"""
        self.move_ptz(0.0, 0.0)

    def stop_then_home_ptz(self):
        """Arrête la PTZ et force le retour HOME via socket direct.
        Utilise un socket direct car le contexte ROS peut être invalide à ce stade."""
        self.shutdown_requested = True
        
        self.get_logger().warn("Envoi de l'ordre HOME via Socket Direct (VISCA)...")

        try:
            import socket
            cam_ip = '192.168.5.163'
            cam_port = 1259 
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                sock.connect((cam_ip, cam_port))
                
                sock.sendall(bytes([0x81, 0x01, 0x06, 0x01, 0x03, 0x03, 0x03, 0x03, 0xFF]))
                _time.sleep(0.1)
                
                sock.sendall(bytes([0x81, 0x01, 0x06, 0x04, 0xFF]))
        except Exception as e:
            self.get_logger().error(f"Erreur Socket: {e}")
    
    def set_auto_exposure(self):
        """Active l'auto-exposure matériel via VISCA"""
        try:
            import socket
            cam_ip = '192.168.5.163'
            cam_port = 1259
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(5.0)
                sock.connect((cam_ip, cam_port))
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
        
        timestamp = _time.strftime("%Y-%m-%d_%H-%M-%S")
        self.video_filename = os.path.join(
            self.video_output_dir,
            f"video_continuous_{timestamp}.mp4"
        )
        
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',
            '-rtsp_transport', 'tcp',
            '-i', self.rtsp_url,
        ]
        
        if self.enable_video_adjustment:
            brightness_ffmpeg = max(-1.0, min(1.0, (self.video_brightness - 1.0) / 2.0))
            ffmpeg_cmd.extend([
                '-vf', f'eq=brightness={brightness_ffmpeg}:contrast={self.video_contrast}:gamma={self.video_gamma}',
                '-c:v', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
                '-movflags', '+faststart',
            ])
        else:
            ffmpeg_cmd.extend([
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-crf', '23',
                '-movflags', '+faststart',
            ])
        
        ffmpeg_cmd.append(self.video_filename)
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.record_start_time = self._now()
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
        recorded_video_path = self.video_filename

        try:
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
        if recorded_video_path and os.path.exists(recorded_video_path):
            self.video_ready_flag_path = f"{recorded_video_path}.done"
            try:
                with open(self.video_ready_flag_path, 'w', encoding='utf-8') as flag_file:
                    flag_file.write(f"{os.path.basename(recorded_video_path)}\n")
                self.get_logger().info(f"Marqueur de fin vidéo créé: {self.video_ready_flag_path}")
            except Exception as e:
                self.get_logger().error(f"Impossible de créer le marqueur de fin vidéo: {e}")

        self.ffmpeg_process = None
        self.video_filename = None
    
    def update_ptz_sweep(self):
        """
        Pattern Scanner de Voûte : Balayage horizontal (Pan) de gauche à droite.
        Le Tilt reste fixé vers le haut pour inspecter la voûte.
        """
        elapsed = self._elapsed_sec(self.ptz_sweep_start_time)
        current_cycle = int(elapsed / self.ptz_sweep_duration)

        frequence = (2 * math.pi) / self.ptz_sweep_duration
        
        amplitude_pan = (self.ptz_end_pan - self.ptz_start_pan) / 2
        
        pan_speed = amplitude_pan * frequence * math.cos(frequence * elapsed)     
        
        tilt_speed = 0.0
        if self.ptz_sweep_progress < 0.1 and current_cycle == 0:
            tilt_speed = 0.1
            
        final_pan_speed = pan_speed * self.ptz_speed_factor
        final_tilt_speed = tilt_speed * self.ptz_speed_factor

        final_pan_speed = max(-1.0, min(1.0, final_pan_speed))
        
        self.move_ptz(final_pan_speed, final_tilt_speed)
        
        self.ptz_sweep_progress = (elapsed % self.ptz_sweep_duration) / self.ptz_sweep_duration
    
    def sequence_loop(self):
        """Boucle principale - mouvement continu avec balayage quart de sphère"""
        if self.state == 'WAITING_START':
            return
        
        elif self.state == 'RECORDING':
            if self.current_position is None:
                return
            
            if self.enable_robot_motion:
                self.move_robot(self.robot_speed)
            
            if not self.ptz_positioning_done:
                pan_to_start = self.ptz_start_pan
                tilt_to_start = self.ptz_start_tilt
                pan_speed_init = 0.5 if pan_to_start > 0 else -0.5 if pan_to_start < 0 else 0.0
                tilt_speed_init = 0.5 if tilt_to_start > 0 else -0.5 if tilt_to_start < 0 else 0.0
                self.move_ptz(pan_speed_init, tilt_speed_init)
                
                if self.positioning_start_time is None:
                    self.ptz_sweep_start_time = self._now()
                    self.positioning_start_time = self._now()
                
                if self._elapsed_sec(self.positioning_start_time) >= 2.0:
                    self.stop_ptz()
                    self.ptz_positioning_done = True
                    self.ptz_sweep_start_time = self._now()
                    self.get_logger().info("PTZ positionnée, début du balayage...")
            
            if self.ptz_positioning_done and not self.shutdown_requested:
                self.update_ptz_sweep()
            
            if self.ffmpeg_process is not None and self.ffmpeg_process.poll() is not None:
                self.get_logger().warn("FFmpeg s'est arrêté de manière inattendue")
                self.stop_video_recording()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSequenceVideo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Arrêt manuel détecté...")
    finally:
        node.stop_then_home_ptz()
        node.stop_robot()
        node.stop_video_recording()
        
        # Délai court pour que les paquets réseau (Socket + FFmpeg) partent
        _time.sleep(0.5)
        
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
