# Description du projet

Ce document explique l'organisation du projet et l'intérêt de chaque dossier.

## Architecture

```
~/Documents/ANDRA_2025-2026/
├── docs/                        # Documentation du projet
├── backup/                      # Empreinte SHA256 de l'image disque usine (voir docs/BACKUP.md)
├── docker/                      # Environnement Docker ROS2 pour visualiser le robot et traiter les images
├── ros2_ws/                     # Workspace principal (code robot, navigation, ros_launcher, etc)
├── dependencies/                # Dépendances externes (workspaces ROS2)
│   ├── ydlidar_ros2_ws/         # Workspace ROS2 pour le lidar YDLidar
│   ├── scout_base/              # Workspace ROS2 pour le robot Scout
│   └── zed-ros2-wrapper/        # Workspace ROS2 pour la caméra ZED2i
├── video/                       # Script d'enregistrement vidéo depuis la caméra PTZ
└── scripts/                     # Scripts d'initialisation et gestion du projet
```

## Description des dossiers

### `docs/`
**Rôle** : Documentation complète du projet

**Contenu** :
- `BACKUP.md` : Sauvegarde et restauration de l’image disque NVMe
- `DEMARRAGE_ROBOT.md` : Guide de démarrage des nœuds sur le robot
- `DEBUG.md` : Commandes de débogage et vérification du système
- `DETECTION_YOLO.md` : Nœud `image_subscriber` sur GPU, modèles PyTorch/TensorRT
- `HOTSPOT.md` : Connexion réseau au robot (Techlab-wifi vs hotspot)
- `PTZ_PRESETS.md` : Guide de contrôle PTZ et utilisation de la caméra
- `SCRIPTS.md` : Documentation détaillée de tous les scripts
- `STRUCTURE.md` : Ce fichier (structure du projet)
- `TRAJECTOIRE_MISSION.md` : Guide de création et exécution des missions par waypoints (AMCL/Nav2)
- `TF_TREE.md` : Arbre TF du robot et repères 
- `VISUALISATION.md` : Guide complet de visualisation avec RViz2 et interprétation des cartes

**Intérêt** : Centralise toute la documentation pour faciliter la compréhension et la maintenance du projet.

---

### `backup/`
**Rôle** : Conserver une **empreinte** pour l’image disque du robot, documentée dans `docs/BACKUP.md`.

**Contenu typique** :
- `backup_robot_usine.img.zst.sha256` : somme SHA256 attendue pour le fichier nommé `backup_robot_usine.img.zst`.

**Ce qui n’est en principe pas dans Git** : le fichier `backup_robot_usine.img.zst` (très volumineux). Une copie pour l’équipe est tenue sur le **drive associé au projet** ; d’autres copies peuvent exister sur disque, NAS ou clé USB. Avant restauration, vérifiez l’intégrité avec `sha256sum` comme dans `docs/BACKUP.md`.

**Intérêt** : Éviter de reflasher ou de conserver une image corrompue ; traçabilité alignée sur la procédure officielle de sauvegarde.

---

### `docker/`
**Rôle** : Environnements Docker pour développement et exécution GPU

**Contenu** :
- `Dockerfile.rviz` : Image Docker avec ROS2 et RViz2 (pour PC de développement)
- `Dockerfile.jetson` : Image Docker avec PyTorch + ROS2 pour Jetson Orin (GPU)
- `launch_rviz.sh` : Script de lancement du conteneur Docker pour PC (RViz2)
- `launch_jetson.sh` : Script de lancement du nœud `image_subscriber` avec GPU sur Jetson
- `README.md` : Notes d’usage des images et scripts Docker

**Intérêt** : 
- Permet de visualiser le robot et la carte depuis un PC distant sans installer ROS2 localement
- Permet d'exécuter le nœud `image_subscriber` avec GPU sur Jetson dans un environnement isolé

---

### `ros2_ws/`
**Rôle** : Workspace ROS2 principal contenant tout le code du projet

**Contenu** :
- `src/image_transfer/` : Package ROS2 pour la détection de fissures et contrôle PTZ
  - `image_publisher` : Capture d'images depuis la caméra PTZ (RTSP)
  - `video_publisher` : Traitement des vidéos enregistrées, extraction d'images et publication sur `/photo_topic`
  - `image_subscriber` : Détection YOLO des fissures sur les images reçues depuis `/photo_topic`
  - `position_publisher` *(WIP, voir ci-dessous)* : devait afficher la position du robot lors des détections
  - `ptz_controller` : Contrôle de la caméra PTZ Marshall CV-605
- `src/navigation_utils/` : Package ROS2 pour les utilitaires de navigation et visualisation
  - `odom_to_path` : Conversion de l'odométrie en Path pour visualisation dans RViz2
  - `report_fissures` *(WIP, voir ci-dessous)* : devait tracer les positions détectées sur la carte
  - `sequence_photo` : Séquence Step-and-Go avec 5 captures PTZ en boucle
  - `sequence_video` : Enregistrement vidéo continu avec balayage PTZ automatique
  - `show_pos` *(WIP, voir ci-dessous)* : devait afficher la position du robot (odométrie)
  - `trajectoire_mission` : Exécute une mission Nav2 à partir d'un fichier de waypoints (`x`, `y`, `yaw`)
  - `trajectoire/` : Dossier des fichiers de trajectoire (`*.yaml`) par carte (ex: `ma_carte_traj.yaml`)

> **Nœuds WIP** : `position_publisher`, `show_pos` et `report_fissures` ne sont pas branchés en production aujourd'hui. La chaîne pose↔détection passe par `position_publisher` qui souscrit à un topic `detection_status` (`std_msgs/Bool`) jamais publié (le détecteur `image_subscriber` publie `/position_detectee` en `geometry_msgs/Point`). Le nœud `report_fissures` a un `map_yaml_path` par défaut pointant vers une carte qui n'existe plus dans `map_results/`. Ces nœuds restent dans le repo pour servir de base à la future tâche TODO « Associer chaque média à la pose robot » mais doivent être considérés comme non fonctionnels tant qu'ils n'ont pas été retravaillés.
- `src/patrouille_autonome/` : Package ROS2 d'orchestration mission + capture autonome
  - `fusion_photo_navigation` : Lance `trajectoire_mission` + `sequence_photo` en parallèle
  - `fusion_video_navigation` : Lance `trajectoire_mission` + `sequence_video` en parallèle (arrêt uniformisé avec `fusion_photo_navigation`, PTZ Home explicitement envoyé par la fusion)
- `src/ros_launcher/` : Package ROS2 de configuration et de lancement (SLAM, AMCL, EKF, Nav2)
  - `navigation_stack.launch.py` : Fichier de lancement principal 
  - `configs/` : Fichiers de configuration YAML
    - `ekf_config.yaml` : Configuration du filtre EKF (fusion des capteurs)
    - `ydlidar_TG15.yaml` : Paramètres YDLidar TG15 custom chargés par défaut par `navigation_stack.launch.py`
    - `slam_config.yaml` : Configuration SLAM (cartographie)
    - `amcl_config.yaml` : Configuration AMCL (localisation)
  - `map_results/` : Cartes créées et sauvegardées (fichiers `.yaml` et `.pgm`)
  - `package.xml` : Manifest du package ROS2
  - `CMakeLists.txt` : Configuration de build
  - `config.rviz` : Configuration pré-définie pour RViz2 
- `images_capturees/` : Images brutes capturées par la caméra PTZ
- `images_detectees/` : Images avec détections de fissures
- `models/` : Modèles YOLO

**Intérêt** : Regroupe l'ensemble du code source du projet (packages dans `src/`, dont `ros_launcher` pour le lancement et les configs). C'est ici que vous développez et modifiez les fonctionnalités du robot.

---

### `dependencies/`
**Rôle** : Workspaces ROS2 des dépendances externes (drivers matériels)

**Contenu** :
- `dependencies.repos` : Fichier `vcs` (URLs et branches) pour les trois workspaces ci-dessous
- `ydlidar_ros2_ws/` : Driver ROS2 pour le LIDAR YDLidar
- `scout_base/` : Driver ROS2 pour le robot Agilex Scout Mini (odométrie des roues)
- `zed-ros2-wrapper/` : Driver ROS2 pour la caméra ZED2i (images et profondeur)

**Intérêt** : Sépare les dépendances externes du code principal. Ces workspaces sont compilés séparément et sourcés par `scripts/setup.sh`. Facilite la mise à jour des drivers sans toucher au code principal.

---

### `video/`
**Rôle** : Script d'enregistrement vidéo depuis la caméra PTZ

**Contenu** :
- `script.sh` : Script interactif d'enregistrement avec retouches d'image
- `video_output/` : Dossier de sauvegarde des vidéos enregistrées

**Intérêt** : Permet d'enregistrer des vidéos depuis le flux RTSP de la caméra PTZ avec des retouches d'image (luminosité, contraste, gamma). Utile pour créer des datasets ou documenter les missions.

---

### `scripts/`
**Rôle** : Scripts d'automatisation pour initialiser, compiler et lancer le système

**Contenu** :
- `setup.sh` : Initialise l'environnement ROS2 et source tous les workspaces
- `build.sh` : Compile les workspaces ROS2 (tous ou un spécifique)
- `launch.sh` : Lance le système complet (mode SLAM ou AMCL)
- `ptz-network-setup.sh` : Configure le réseau Ethernet pour accéder à la caméra PTZ
- `image_subscriber_gpu.sh` : Lance le nœud `image_subscriber` avec GPU dans Docker
- `convert_to_tensorrt.sh` : Convertit le modèle PyTorch (`best.pt`) en TensorRT (`best.engine`) pour améliorer les performances

**Intérêt** : Simplifie grandement l'utilisation du projet. Au lieu de se souvenir de multiples commandes ROS2, vous utilisez des scripts simples. Facilite l'onboarding de nouveaux utilisateurs.
