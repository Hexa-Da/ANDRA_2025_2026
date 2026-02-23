# ROS2 Docker - Guide d'utilisation

## Vue d'ensemble

Ce dossier contient les environnements Docker pour le projet. **Tout est déjà configuré** : les images Docker sont construites et prêtes à l'emploi.

**Deux environnements disponibles** :
1. **PC de développement** : Visualiser avec RViz2 les cartes SLAM et les données du robot
2. **Jetson Orin** : Exécuter le nœud `image_subscriber` avec PyTorch et GPU pour la détection YOLO

---

## Environnement 1 : PC de développement (RViz2)

### À quoi ça sert

**Visualiser le robot à distance sans installer ROS2 sur votre PC.**

Permet de :
- Visualiser les cartes SLAM créées par le robot
- Voir les données du robot (LIDAR, position, trajectoire) en temps réel
- Déboguer visuellement le système robotique

### Utilisation

**Lancer le conteneur** :
```bash
cd ros2_docker
./launch_rviz.sh
```

**Dans le conteneur** :
```bash
# Compiler le workspace (si nécessaire)
colcon build

# Sourcer le workspace
source install/setup.bash

# Lancer RViz2
rviz2 -d /workspace/ros2_ws/src/ros_launcher/config.rviz
```

**Prérequis** :
- Docker installé
- Linux avec X11 (pour RViz2)
- Même réseau WiFi que le robot
- Workspace ROS2 compilé sur le robot

---

## Environnement 2 : Jetson Orin (PyTorch + GPU)

### À quoi ça sert

**Exécuter le nœud `image_subscriber` avec GPU pour la détection YOLO des fissures.**

### Utilisation

**Lancer le nœud** (depuis la racine du projet) :
```bash
./scripts/image_subscriber_gpu.sh
```

**Remarque importante** :  
L'utilisation de cet environnement ne se fait **pas** en lançant des commandes depuis ce dossier `ros2_docker/`, mais via le script `./scripts/image_subscriber_gpu.sh`. Ce script utilise les fichiers Docker présents dans ce dossier pour démarrer (`launch_jetson.sh`) un conteneur adapté à Jetson (`Dockerfile.jetson`), permettant l'exécution du nœud `image_subscriber` avec GPU.

---

## Structure des fichiers

### Dockerfiles

- **`Dockerfile.rviz`** : Image pour PC de développement (ROS2 + RViz2)
- **`Dockerfile.jetson`** : Image pour Jetson (PyTorch + ROS2 + GPU)

**Note** : Les images sont déjà construites. Pour reconstruire l'image PC : `docker build -t ros2-humble-custom -f Dockerfile.rviz .`

### Scripts de lancement

- **`launch_rviz.sh`** : Lance le conteneur PC pour RViz2
- **`launch_jetson.sh`** : Lance le nœud `image_subscriber` avec GPU sur Jetson
