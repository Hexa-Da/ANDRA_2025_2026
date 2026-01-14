# ROS2 Docker - Guide d'utilisation

## Objectif

Ce dossier (`ros-docker/`) permet de travailler et tester votre code ROS2 sans avoir accès au robot physique. 

C'est l'environnement de développement local qui vous permet de :
- **Développer et tester vos nœuds ROS2** sur votre machine personnelle 
- **Verifier que votre code compile** avant de le déployer sur le robot
- **Visualiser avec RViz2** les données du robot (si connecté au même réseau)
- **Travailler sur n'importe quelle distribution Linux** (alors que ROS2 Humble est uniquement sur Ubuntu 22.04)


## Prérequis

- **Docker** installé et fonctionnel
- **Environnement X11** (pour RViz2)
  - **Linux** : X11 natif (déjà installé)
  - **macOS** : **XQuartz** doit être installé et démarré (voir section macOS ci-dessous)
- **Pas besoin d'installer ROS2 nativement** sur votre système

## Installation (une seule fois)

```bash
cd ros-docker
docker build -t ros2-humble-custom .
```

Cette commande va :
- Télécharger l'image de base `ros:humble-ros-core-jammy` (Ubuntu 22.04)
- Installer tous les outils de développement ROS2
- Installer les dépendances Python (numpy<2, ultralytics, pillow, matplotlib)
- Installer les paquets ROS2 nécessaires (cv-bridge, image-geometry, rviz2)

## Structure des fichiers

### `Dockerfile`
Crée une image Docker complète avec :
- **Image de base** : `ros:humble-ros-core-jammy` (Ubuntu 22.04 + ROS2 Humble core)
- **Outils de développement** : build-essential, git, colcon, rosdep, vcstool
- **Paquets ROS2** : ros-base, cv-bridge, image-geometry, rviz2
- **Dépendances Python** : numpy<2 (compatibilité cv_bridge), ultralytics (YOLOv11), pillow, matplotlib
- **Outils multimédias** : opencv, ffmpeg

### `launch.sh` (Linux)
Script de lancement du conteneur Docker pour Linux avec :
- **Accès X11** : pour afficher RViz2 (`xhost +local:`)
- **Réseau hôte** : `--net=host` pour communiquer avec le robot (si sur le même réseau)
- **Volumes montés** :
  - `/tmp/.X11-unix` → accès à l'affichage graphique
  - `../ros2_ws` → workspace ROS2 (monté en `/workspace/ros2_ws`)
  - `../ros_launcher` → fichiers de lancement (monté en `/workspace/ros_launcher`)

### `launch-macos.sh` (macOS)
Script de lancement du conteneur Docker pour macOS avec :
- **Accès XQuartz** : configuration automatique pour l'affichage graphique
- **Détection automatique** : vérifie que XQuartz est démarré
- **Même configuration réseau et volumes** que `launch.sh`

## Utilisation

### macOS - Configuration spéciale

**Sur macOS, vous devez utiliser XQuartz pour l'affichage graphique :**

1. **Installer XQuartz** (si ce n'est pas déjà fait) :
   ```bash
   # Via Homebrew
   brew install --cask xquartz
   
   # Ou télécharger depuis: https://www.xquartz.org/
   ```

2. **Démarrer XQuartz** :
   - Ouvrir **Applications > Utilitaires > XQuartz**
   - Ou depuis le terminal : `open -a XQuartz`

3. **Configurer XQuartz** pour accepter les connexions réseau :
   - Dans XQuartz : **Préférences > Sécurité**
   - Cocher **"Autoriser les connexions depuis le réseau"**
   - Redémarrer XQuartz si nécessaire

4. **Lancer le conteneur avec le script macOS** :
   ```bash
   cd ros-docker
   ./launch-macos.sh
   ```

### Linux - Utilisation standard

```bash
cd ros-docker
./launch.sh
```

### Ce qui se passe lors du lancement

1. Vérification de l'environnement X11 (XQuartz sur macOS)
2. Autorisation de l'accès X11
3. Lancement d'un conteneur interactif avec bash
4. Montage des workspaces en volumes

### Dans le conteneur

**Workflow de développement :**

```bash
# 1. Initialiser ROS2 (déjà fait automatiquement par le script)
source /opt/ros/humble/setup.bash

# 2. Aller dans le workspace
cd /workspace/ros2_ws

# 3. Compiler le workspace (si nécessaire)
# Équivalent à ./scripts/build.sh andra sur le robot
colcon build

# 4. Sourcer le workspace compilé
source install/setup.bash

# 5. Lancer RViz2 avec votre configuration pour visualiser les cartes SLAM
rviz2 -d /workspace/ros_launcher/config.rviz

# 6. Tester vos nodes ROS2 individuellement
ros2 run image_transfer image_publisher
ros2 run image_transfer image_subscriber
# etc.
```

**💡 Pour visualiser les cartes créées par SLAM :**
```bash
# Dans le conteneur, une fois le système lancé sur le robot
rviz2 -d /workspace/ros_launcher/config.rviz
```

Le fichier `config.rviz` est automatiquement monté depuis `ros_launcher/config.rviz` et contient toute la configuration nécessaire pour visualiser :
- La carte (`/map`)
- Les scans LIDAR (`/scan`)
- Les transformations TF
- La position du robot
- Les particules AMCL (en mode localisation)

**Note** : Dans Docker, vous devez compiler manuellement avec `colcon build`. Les scripts `scripts/build.sh` et `scripts/setup.sh` sont conçus pour fonctionner directement sur le robot (voir [`DEMARRAGE_ROBOT.md`](../DEMARRAGE_ROBOT.md) pour le workflow complet sur le robot).

## Workflow de développement

### Édition des fichiers

**Vous pouvez éditer vos fichiers SANS lancer le conteneur** :
- Modifier les fichiers Python dans `ros2_ws/src/`
- Modifier les fichiers YAML dans `ros_launcher/`
- Les changements sont immédiatement visibles dans le conteneur (grâce aux volumes montés)

### Compilation et exécution

**Lancez le conteneur uniquement pour** :
- Compiler le projet (`colcon build`)
- Lancer des nodes ROS2 (`ros2 launch`, `ros2 run`)
- Visualiser avec RViz2
- Tester votre code localement

### Connexion au robot (optionnel)

Si le robot est sur le même réseau et que vous voulez visualiser ses données :
- Le conteneur utilise `--net=host` pour accéder au réseau
- Vous pouvez lancer RViz2 et vous connecter aux topics du robot
- Voir `DEMARRAGE_ROBOT.md` pour savoir comment lancer le système sur le robot

## Note technique

Développer dans un conteneur Docker offre des possibilités limitées par rapport à un accès direct au robot via SSH. N’utilisez cette méthode que si l’accès physique ou distant au robot n’est vraiment pas possible. Il est fortement recommandé de faire le développement directement sur le robot pour bénéficier de toutes les fonctionnalités et simplifier le processus.

## Liens utiles

- [Documentation ROS2 Humble](https://docs.ros.org/en/humble/)
- [Documentation Docker](https://docs.docker.com/)

