import rclpy
from rclpy.node import Node
import cv2
import os
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class VideoFilePublisher(Node):
    def __init__(self):
        super().__init__('video_file_publisher')
        self.publisher = self.create_publisher(Image, 'photo_topic', 10)
        self.bridge = CvBridge()

        # Configuration des dossiers
        project_root = os.path.expanduser("~/Documents/ANDRA_2025-2026")
        self.video_input_dir = os.path.join(project_root, "video/video_output")
        self.images_output_dir = os.path.join(project_root, "ros2_ws/images_capturees")
        self.processed_dir = os.path.join(self.video_input_dir, "processed")
        self.failed_dir = os.path.join(self.video_input_dir, "failed")  # Pour les vidéos corrompues

        # extract_rate: nombre d'images extraites par seconde de vidéo
        self.declare_parameter('extract_rate', 30.0)
        self.extract_rate = self.get_parameter('extract_rate').get_parameter_value().double_value
        self.get_logger().info(f"Taux d'extraction: {self.extract_rate} images/seconde")

        # Mode optionnel: traiter ponctuellement puis quitter
        self.declare_parameter('run_once', False)
        self.declare_parameter('run_once_timeout_sec', 45.0)
        self.declare_parameter('scan_period_sec', 5.0)
        self.declare_parameter('validation_retry_count', 5)
        self.declare_parameter('validation_retry_delay_sec', 1.0)
        # Log tous les N images sauvegardées pendant process_video (0 = désactivé)
        self.declare_parameter('progress_log_every_saved_frames', 5)
        self.run_once = self.get_parameter('run_once').get_parameter_value().bool_value
        self.run_once_timeout_sec = self.get_parameter('run_once_timeout_sec').get_parameter_value().double_value
        configured_scan_period = self.get_parameter('scan_period_sec').get_parameter_value().double_value
        self.validation_retry_count = max(
            1, self.get_parameter('validation_retry_count').get_parameter_value().integer_value
        )
        self.validation_retry_delay_sec = max(
            0.1, self.get_parameter('validation_retry_delay_sec').get_parameter_value().double_value
        )
        self.progress_log_every_saved_frames = max(
            0, self.get_parameter('progress_log_every_saved_frames').get_parameter_value().integer_value
        )
        self.scan_period_sec = max(0.2, configured_scan_period)
        if self.run_once:
            self.scan_period_sec = min(self.scan_period_sec, 1.0)
        self.run_once_start = time.time()
        self.should_exit = False
        
        # Création des dossiers
        os.makedirs(self.video_input_dir, exist_ok=True)  # Créer le dossier d'entrée s'il n'existe pas
        os.makedirs(self.images_output_dir, exist_ok=True)
        os.makedirs(self.processed_dir, exist_ok=True)
        os.makedirs(self.failed_dir, exist_ok=True)

        # Timer périodique de scan
        self.timer = self.create_timer(self.scan_period_sec, self.check_for_videos)
        mode = "ponctuel" if self.run_once else "continu"
        self.get_logger().info(
            f"Surveillance du dossier ({mode}, période {self.scan_period_sec:.1f}s) : {self.video_input_dir}"
        )
        
        # Dictionnaire pour suivre la taille des fichiers (pour détecter les fichiers en cours d'écriture)
        self.file_sizes = {}
        # Compteur d'échecs de validation par vidéo, pour étaler les retry sur plusieurs ticks
        # (évite tout time.sleep dans le callback du timer ROS).
        self.validation_failures = {}

    def is_video_marked_ready(self, video_path):
        """Vérifie que la vidéo est marquée comme terminée par sequence_video.py."""
        ready_flag_path = f"{video_path}.done"
        return os.path.exists(ready_flag_path)

    def consume_video_ready_flag(self, video_path):
        """Supprime le marqueur .done après traitement de la vidéo."""
        ready_flag_path = f"{video_path}.done"
        if os.path.exists(ready_flag_path):
            try:
                os.remove(ready_flag_path)
            except Exception as e:
                self.get_logger().warn(f"Impossible de supprimer le marqueur {os.path.basename(ready_flag_path)}: {e}")

    def is_file_stable(self, video_path, min_age_seconds=2.0):
        """Vérifie si un fichier est stable (pas en cours d'écriture)"""
        try:
            current_size = os.path.getsize(video_path)
            current_mtime = os.path.getmtime(video_path)
            
            # Vérifier si le fichier est assez récent (peut être encore en cours d'écriture)
            time_since_modification = time.time() - current_mtime
            if time_since_modification < min_age_seconds:
                return False  # Fichier trop récent, probablement encore en cours d'écriture
            
            # Vérifier si la taille a changé depuis la dernière vérification
            if video_path in self.file_sizes:
                if self.file_sizes[video_path] == current_size:
                    return True  # Taille stable, fichier probablement complet
                else:
                    # Taille a changé, mettre à jour et retourner False
                    self.file_sizes[video_path] = current_size
                    return False
            else:
                # Première vérification, enregistrer la taille
                self.file_sizes[video_path] = current_size
                return False  # Attendre une deuxième vérification
        except Exception as e:
            self.get_logger().warn(f"Erreur lors de la vérification de stabilité de {os.path.basename(video_path)}: {e}")
            return False

    def is_video_valid(self, video_path):
        """Vérifie si la vidéo est valide et peut être lue"""
        try:
            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                cap.release()
                return False
            
            # Vérifier que le FPS est valide
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps <= 0:
                cap.release()
                return False
            
            # Essayer de lire au moins une frame
            ret, frame = cap.read()
            cap.release()
            
            if not ret or frame is None:
                return False
            
            return True
        except Exception as e:
            self.get_logger().warn(f"Erreur lors de la validation de {os.path.basename(video_path)}: {e}")
            return False

    def validate_or_defer(self, video_path):
        """Retourne 'valid', 'defer' ou 'failed' sans bloquer le timer ROS.

        Les retries sont étalés sur plusieurs ticks du timer (validation_retry_count
        échecs cumulés avant d'abandonner). Cela remplace l'ancienne boucle
        time.sleep qui bloquait l'exécuteur ROS.
        """
        if self.is_video_valid(video_path):
            attempts = self.validation_failures.pop(video_path, 0)
            if attempts > 0:
                self.get_logger().info(
                    f"Vidéo valide après {attempts + 1} tentative(s): {os.path.basename(video_path)}"
                )
            return 'valid'

        attempts = self.validation_failures.get(video_path, 0) + 1
        self.validation_failures[video_path] = attempts

        if attempts < self.validation_retry_count:
            self.get_logger().warn(
                f"Validation échouée ({attempts}/{self.validation_retry_count}) "
                f"pour {os.path.basename(video_path)}; nouvelle tentative au prochain scan"
            )
            return 'defer'

        self.validation_failures.pop(video_path, None)
        return 'failed'

    def check_for_videos(self):
        try:
            if not os.path.exists(self.video_input_dir):
                self.get_logger().warn(f"Le dossier {self.video_input_dir} n'existe pas")
                return
            
            videos = [f for f in os.listdir(self.video_input_dir) if f.endswith('.mp4')]
            ready_videos = 0
            processed_count = 0
            
            if not videos:
                if self.run_once and (time.time() - self.run_once_start) >= self.run_once_timeout_sec:
                    self.get_logger().warn("Aucune vidéo détectée avant timeout en mode run_once")
                    self.should_exit = True
                return

            self.get_logger().info(f"Vidéos trouvées : {len(videos)} fichier(s)")

            for video_file in videos:
                video_path = os.path.join(self.video_input_dir, video_file)

                # Ne traiter que les vidéos marquées "terminées" par sequence_video.py
                if not self.is_video_marked_ready(video_path):
                    continue
                ready_videos += 1
                
                # Vérifier si le fichier est stable (pas en cours d'écriture)
                if not self.is_file_stable(video_path, min_age_seconds=2.0):
                    # Nettoyer l'entrée du dictionnaire si le fichier n'existe plus
                    if video_path in self.file_sizes and not os.path.exists(video_path):
                        del self.file_sizes[video_path]
                    continue  # Attendre que le fichier soit stable
                
                # Vérifier si la vidéo est valide avant de la traiter
                validation_result = self.validate_or_defer(video_path)
                if validation_result == 'defer':
                    continue
                if validation_result == 'failed':
                    self.get_logger().warn(
                        f"Vidéo corrompue ou invalide détectée : {video_file}. Déplacement vers 'failed'"
                    )
                    failed_path = os.path.join(self.failed_dir, video_file)
                    try:
                        os.rename(video_path, failed_path)
                        self.consume_video_ready_flag(video_path)
                    except Exception as e:
                        self.get_logger().error(f"Impossible de déplacer {video_file} vers failed: {e}")
                    continue
                
                self.get_logger().info(
                    f"État traitement : {video_file} stable et validée, début du découpage"
                )
                # Traiter la vidéo valide
                success = self.process_video(video_path, video_file)
                
                # Nettoyer l'entrée du dictionnaire après traitement
                if video_path in self.file_sizes:
                    del self.file_sizes[video_path]
                
                # Déplacer la vidéo après traitement (succès ou échec)
                if success:
                    processed_path = os.path.join(self.processed_dir, video_file)
                    try:
                        os.rename(video_path, processed_path)
                        self.consume_video_ready_flag(video_path)
                        self.get_logger().info(
                            f"Vidéo traitée avec succès et déplacée vers : {processed_path}"
                        )
                        processed_count += 1
                    except Exception as e:
                        self.get_logger().error(f"Impossible de déplacer {video_file} vers processed: {e}")
                else:
                    failed_path = os.path.join(self.failed_dir, video_file)
                    try:
                        os.rename(video_path, failed_path)
                        self.consume_video_ready_flag(video_path)
                        self.get_logger().warn(
                            f"Échec du traitement, vidéo déplacée vers : {failed_path}"
                        )
                        processed_count += 1
                    except Exception as e:
                        self.get_logger().error(f"Impossible de déplacer {video_file} vers failed: {e}")

            if self.run_once:
                elapsed = time.time() - self.run_once_start
                if processed_count > 0:
                    self.get_logger().info(
                        f"Mode run_once: {processed_count} vidéo(s) traitée(s), arrêt du nœud"
                    )
                    self.should_exit = True
                elif ready_videos > 0 and elapsed >= self.run_once_timeout_sec:
                    self.get_logger().warn(
                        f"Mode run_once: timeout après {elapsed:.1f}s sans traitement finalisé, arrêt du nœud"
                    )
                    self.should_exit = True
                elif ready_videos == 0 and elapsed >= self.run_once_timeout_sec:
                    self.get_logger().warn(
                        f"Mode run_once: aucune vidéo prête (.done) après {elapsed:.1f}s, arrêt du nœud"
                    )
                    self.should_exit = True
                        
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la vérification des vidéos : {e}")

    def process_video(self, path, filename):
        """Traite une vidéo et retourne True si succès, False sinon"""
        try:
            self.get_logger().info(f"Traitement de la vidéo : {filename}")
            cap = cv2.VideoCapture(path)
            
            if not cap.isOpened():
                self.get_logger().error(f"Impossible d'ouvrir la vidéo : {filename}")
                return False
            
            fps = cap.get(cv2.CAP_PROP_FPS)
            
            if fps == 0: 
                self.get_logger().error(f"Impossible de lire le FPS de {filename} (vidéo corrompue ou incomplète)")
                cap.release()
                return False

            hop_frames = max(1, int(fps / self.extract_rate))  # Éviter division par zéro
            self.get_logger().info(
                f"Découpage {filename}: fps≈{fps:.2f}, extract_rate={self.extract_rate}, hop_frames={hop_frames}"
            )
            frame_count = 0
            saved_count = 0

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                # Extraire seulement aux intervalles définis
                if frame_count % hop_frames == 0:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    img_name = f"from_vid_{timestamp}_{saved_count:03d}.jpg"
                    img_path = os.path.join(self.images_output_dir, img_name)

                    # 1. Sauvegarde physique dans images_capturees
                    cv2.imwrite(img_path, frame)
                    
                    # Publication ROS sur /photo_topic pour image_subscriber (YOLO)
                    try:
                        ros_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.publisher.publish(ros_img)
                    except Exception as e:
                        self.get_logger().warn(f"Erreur publication image {saved_count}: {e}")
                    
                    saved_count += 1
                    every = self.progress_log_every_saved_frames
                    if every > 0 and saved_count % every == 0:
                        self.get_logger().info(
                            f"Progression {filename}: {saved_count} image(s) sauvegardée(s), "
                            f"frame lue={frame_count}"
                        )

                frame_count += 1

            cap.release()
            
            if saved_count == 0:
                self.get_logger().warn(f"Aucune image extraite de {filename}")
                return False
            
            self.get_logger().info(f"Terminé : {saved_count} images extraites de {filename}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement de {filename}: {e}")
            if 'cap' in locals():
                cap.release()
            return False

def main(args=None):
    rclpy.init(args=args)
    node = VideoFilePublisher()
    try:
        if node.run_once:
            while rclpy.ok() and not node.should_exit:
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()