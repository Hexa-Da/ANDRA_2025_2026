# Decription du projetription

Ce document explique l'organisation du projet et l'intérêt de chaque dossier.

## Architecture

```
~/Documents/ANDRA_2025-2026/
├── docs/                        # Documentation du projet
├── ros-docker/                  # Environnement Docker ROS2 pour developper sans avoir accès au robot
├── ros2_ws/                     # Workspace principal (code robot, navigation, etc)
├── ros_launcher/                # Fichiers de lancement ROS2 (scripts .launch.py et cartes)
├── dependencies/                # Dépendances externes (workspaces ROS2)
│   ├── ydlidar_ros2_ws/         # Workspace ROS2 pour le lidar YDLidar
│   ├── scout_base/              # Workspace ROS2 pour le robot Scout
│   └── zed-ros2-wrapper/        # Workspace ROS2 pour la caméra ZED2
├── video_launcher/              # Script d'enregistrement vidéo depuis la caméra PTZ
└── scripts/                     # Scripts d'initialisation et gestion du projet
```

## Description des dossiers

### `docs/`
**Rôle** : Documentation complète du projet

**Contenu** :
- `DEMARRAGE_ROBOT.md` : Guide de démarrage des nœuds sur le robot
- `DEBUG.md` : Commandes de débogage et vérification du système
- `SCRIPTS.md` : Documentation détaillée de tous les scripts
- `DESCRIPTION.md` : Ce fichier (structure du projet)

**Intérêt** : Centralise toute la documentation pour faciliter la compréhension et la maintenance du projet.

---

### `ros-docker/`
**Rôle** : Environnement Docker pour visualiser le robot à distance

**Contenu** :
- `Dockerfile` : Image Docker avec ROS2 et RViz2
- `config.rviz` : Configuration RViz2 pour visualiser le robot et la carte
- `launch.sh` : Script de lancement du conteneur Docker

**Intérêt** : Permet de visualiser le robot et la carte depuis un PC distant sans installer ROS2 localement.

---

### `ros2_ws/`
**Rôle** : Workspace ROS2 principal contenant tout le code du projet

**Contenu** :
- `src/image_transfer/` : Package ROS2 pour la détection de fissures
  - `image_publisher` : Capture d'images depuis la caméra PTZ
  - `image_subscriber` : Détection YOLO des fissures
  - `position_publisher` : Affichage de la position du robot
  - `report_fissures` : Traçage des positions détectées sur la carte
  - `ptz_controller` : Contrôle de la caméra PTZ
  - `resource/` : Fichier de marqueur requis par ament_index
  - `test/` : Tests de qualité de code (linting, copyright, PEP257) - standards ROS2
  - `package.xml` : Métadonnées du package ROS2 (dépendances, version, description) 
  - `setup.py` : Configuration Python setuptools (entry points, dépendances Python) 
  - `setup.cfg` : Configuration additionnelle setuptools (emplacement des scripts) 
- `src/slam_andra_package/` : Package de configuration SLAM
- `src/ydlidar_nav2_slam/` : Configuration SLAM pour le LIDAR
- `images_capturees/` : Images brutes capturées par la caméra PTZ
- `images_detectees/` : Images avec détections de fissures
- `models/` : Modèles YOLO

**Intérêt** : Contient tout le code source du projet. C'est ici que vous développez et modifiez les fonctionnalités du robot.

---

### `ros_launcher/`
**Rôle** : Fichiers de configuration et de lancement du système ROS2

**Contenu** :
- `navigation_stack.launch.py` : Fichier de lancement principal (lance tous les nœuds)
- `configs/` : Fichiers de configuration YAML
  - `ekf_config.yaml` : Configuration du filtre EKF (fusion des capteurs)
  - `slam_config.yaml` : Configuration SLAM (cartographie)
  - `amcl_config.yaml` : Configuration AMCL (localisation)
- `map_results/` : Cartes créées et sauvegardées (fichiers `.yaml` et `.pgm`)
- `ydlidar_config.yaml` : Configuration du LIDAR

**Intérêt** : Centralise toutes les configurations et permet de lancer le système complet avec une seule commande. Les cartes sont sauvegardées ici pour être réutilisées en mode AMCL.

---

### `dependencies/`
**Rôle** : Workspaces ROS2 des dépendances externes (drivers matériels)

**Contenu** :
- `ydlidar_ros2_ws/` : Driver ROS2 pour le LIDAR YDLidar
- `scout_base/` : Driver ROS2 pour le robot Agilex Scout Mini (odométrie des roues)
- `zed-ros2-wrapper/` : Driver ROS2 pour la caméra ZED2 (images et profondeur)

**Intérêt** : Sépare les dépendances externes du code principal. Ces workspaces sont compilés séparément et sourcés par `scripts/setup.sh`. Facilite la mise à jour des drivers sans toucher au code principal.

---

### `video_launcher/`
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

**Intérêt** : Simplifie grandement l'utilisation du projet. Au lieu de se souvenir de multiples commandes ROS2, vous utilisez des scripts simples. Facilite l'onboarding de nouveaux utilisateurs.
