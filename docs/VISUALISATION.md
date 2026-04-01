# Guide de visualisation avec RViz2

Ce document explique comment utiliser RViz2 pour visualiser le robot et observer son comportement, avec ou sans carte.

## Qu'est-ce que RViz2 ?

RViz (Robot Visualization) est l'outil de visualisation 3D de ROS2 qui permet de :
- **Visualiser les données des capteurs** (LIDAR, caméras)
- **Observer la position du robot** et ses déplacements
- **Voir la carte** générée par SLAM ou chargée (si disponible)
- **Définir des objectifs de navigation** (Set Goal)
- **Déboguer visuellement** le système robotique

## Configuration : `ros2_ws/src/ros_launcher/config.rviz`

Le fichier `ros2_ws/src/ros_launcher/config.rviz` contient une configuration pré-définie qui affiche tous les éléments essentiels.

## Deux modes d'utilisation

### Mode 1 : Sans carte (observation des dérives)

**Quand l'utiliser ?**
- Pour observer les dérives entre les différents repères du robot
- Pour déboguer l'odométrie sans avoir besoin d'une carte
- Pour tester le système avant de créer une carte

**Configuration :**
- **Fixed Frame** : `odom` (repère local continu, recommandé sans carte)
- **Vue** : Ne suit pas le robot (reste centrée sur l'origine)

**Ce que vous observez :**
- La position du robot dans un repère fixe
- Les dérives de l'odométrie (si `odom` ou `base_link` se déplace alors que le robot est immobile)
- Les différences entre les frames (`odom`, `base_link`, etc.)

**Frames disponibles :**
- `odom` : Repère de l'odométrie (publié par EKF)
- `base_link` : Repère du robot (centre du robot)
- `base_footprint` : Projection au sol du robot
- `laser_frame` : Repère du LIDAR
- `zed_camera_link` : Repère de la caméra ZED2i

### Mode 2 : Avec carte (navigation)

**Quand l'utiliser ?**
- Pour créer une carte avec SLAM
- Pour naviguer sur une carte existante avec AMCL
- Pour planifier des trajectoires

**Configuration :**
- **Fixed Frame** : `map` (repère de la carte) 
- **Vue** : Suit le robot (`base_link`)

**Ce que vous observez :**
- La carte générée par SLAM ou chargée par AMCL
- La position du robot sur la carte
- Le trajet parcouru par le robot
- Les scans LIDAR superposés à la carte

**Frames disponibles :**
- `map` : Repère de la carte globale (publié par SLAM/AMCL)
- `odom` : Repère de l'odométrie (publié par EKF)
- `base_link` : Repère du robot (centre du robot)
- `base_footprint` : Projection au sol du robot 
- `laser_frame` : Repère du LIDAR
- `zed_camera_link` : Repère de la caméra ZED2i

## Displays configurés dans RViz

### Éléments toujours visibles

#### 1. **Grille** (Grid)
- Référence visuelle pour l'orientation
- **Reference Frame** : suit `odom` ou `map` selon le **Fixed Frame** global
- **Offset Z** : `-0.178` m (alignement visuel avec le plan **sol / empreinte** )
- Grille 2D dans le plan XY du repère choisi, puis décalée selon l’offset

#### 2. **TF (Transform)**
- Affiche l'arbre de coordonnées du robot
- Montre tous les frames et leurs relations
- **Utilité** : Vérifier que toutes les transformations sont correctement publiées

#### 3. **LaserScan**
- **Topic** : `/scan`
- Affiche les données du LIDAR en temps réel
- Points colorés selon l'intensité du signal
- **Repère d’affichage** : avec RViz2 (plugin Humble), le scan est transformé dans le **Fixed Frame** (`odom` ou `map`).
- **Utilité** : Vérifier que le LIDAR fonctionne correctement

#### 4. **Path** (Trajet du robot)
- **Topic** : `/robot_path`
- Affiche le trajet parcouru par le robot
- Ligne orange montrant l'historique des positions
- **Utilité** : Visualiser le chemin emprunté par le robot

### Éléments visibles uniquement avec carte

#### 5. **Carte** (Map)
- **Topic** : `/map`
- Affiche la carte générée par SLAM ou chargée par AMCL
- **Visualisation** :
  - **Blanc** : Espace libre (traversable)
  - **Noir** : Obstacles (murs, objets)
  - **Gris** : Espace inconnu

#### 6. **PoseArray** (Particules AMCL)
- **Topic** : `/particlecloud`
- Affiche les particules de localisation AMCL
- Flèches représentant les hypothèses de position
- **Utilité** : Voir la convergence des particules (mode AMCL uniquement)

#### 7. **Pose** (Position estimée)
- **Topic** : `/amcl_pose`
- Affiche la position estimée du robot par AMCL
- Flèche indiquant la position et l'orientation
- **Utilité** : Vérifier la précision de la localisation (mode AMCL uniquement)

#### 8. **Polygon** (Empreinte du robot)
- **Topic** : `/local_costmap/published_footprint`
- Affiche l'empreinte du robot sur la carte
- **Utilité** : Vérifier que le robot peut passer dans les passages

## Utilisation

### Lancer RViz2

1. **Démarrer les nœuds ROS2 sur le robot** :
   ```bash
   # Sur le robot
   ssh techlab@orin2.local
   cd ~/Documents/ANDRA_2025-2026
   source scripts/setup.sh
   # Mode SLAM pour créer une carte ou pour observer les derives en desactivant les capteurs
   ./scripts/launch.sh slam  
   # Mode AMCL pour naviguer sur une carte existante
   ./scripts/launch.sh amcl  
   ```

2. **Lancer RViz2 sur votre PC Linux** :
   ```bash
   # Sur une machine Linux du TechLab
   cd docker
   ./launch_rviz.sh  # Lance le conteneur Docker avec --net=host
   # Puis dans le conteneur 
   rviz2 -d ros2_ws/src/ros_launcher/config.rviz
   ```

   **Note importante** : Le conteneur Docker utilise `--net=host` pour communiquer avec le robot. Assurez-vous que :
   - Votre machine Linux est sur le même réseau WiFi que le robot
   - `ROS_DOMAIN_ID=0` est défini (défini automatiquement par `scripts/setup.sh` sur le robot)
   - Le middleware DDS est cohérent (FastRTPS par défaut)

3. **Observer la visualisation** :
   - Les scans LIDAR apparaissent en temps réel
   - Le robot est représenté par ses frames TF
   - Le trajet du robot est visualisé via le Path `/robot_path` (ligne orange)
   - Si une carte est disponible, elle s'affiche en arrière-plan

4. **Interagir avec le robot** (mode avec carte uniquement) :
   - Utiliser "2D Goal Pose" pour envoyer le robot à un endroit précis
   - Utiliser "2D Pose Estimate" pour corriger la position estimée (mode AMCL)

## Sauvegarde de la carte

Une fois la carte construite en mode SLAM, vous pouvez la sauvegarder :

```bash
# Sur le robot
cd ~/Documents/ANDRA_2025-2026
ros2 run nav2_map_server map_saver -f ros2_ws/src/ros_launcher/map_results/ma_carte
```

Cela créera deux fichiers :
- `ma_carte.yaml` : Métadonnées de la carte
- `ma_carte.pgm` : Image de la carte (noir et blanc)

