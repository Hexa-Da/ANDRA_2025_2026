# Guide de visualisation avec RViz2

Ce document explique comment utiliser RViz2 pour visualiser le robot, les cartes générées par SLAM, et toutes les données des capteurs.

## Qu'est-ce que RViz2 ?

RViz (Robot Visualization) est l'outil de visualisation 3D de ROS2 qui permet de :
- **Voir la carte** générée par le SLAM en temps réel
- **Visualiser les données des capteurs** (LIDAR, caméras)
- **Observer la position du robot** sur la carte
- **Définir des objectifs de navigation** (Set Goal)
- **Déboguer visuellement** le système robotique

## Configuration : `ros_launcher/config.rviz`

Le fichier `ros_launcher/config.rviz` contient une configuration pré-définie qui affiche tous les éléments essentiels pour visualiser le robot et la carte.

### Displays configurés

#### 1. **Grille** (Grid)
- **Rôle** : Référence visuelle pour l'orientation
- **Configuration** : Grille 2D dans le plan XY
- **Utilité** : Aide à comprendre l'échelle et l'orientation de la carte

#### 2. **TF (Transform)**
- **Rôle** : Affiche l'arbre de coordonnées du robot
- **Frames affichées** :
  - `map` : Repère de la carte globale (publié par SLAM/AMCL)
  - `odom` : Repère de l'odométrie (publié par EKF)
  - `base_link` : Repère du robot (centre du robot)
  - `laser_frame` : Repère du LIDAR
  - `zed_camera_link` : Repère de la caméra ZED2
- **Utilité** : Vérifier que toutes les transformations sont correctement publiées

#### 3. **Carte** (Map)
- **Topic** : `/map`
- **Rôle** : Affiche la carte générée par SLAM ou chargée par AMCL
- **Visualisation** :
  - **Blanc** : Espace libre (traversable)
  - **Noir** : Obstacles (murs, objets)
  - **Gris** : Espace inconnu
- **Utilité** : Voir la carte se construire en temps réel (SLAM) ou vérifier la localisation (AMCL)

#### 4. **LaserScan**
- **Topic** : `/scan`
- **Rôle** : Affiche les données du LIDAR en temps réel
- **Visualisation** : Points colorés selon l'intensité du signal
- **Utilité** : Vérifier que le LIDAR fonctionne et voit correctement l'environnement

#### 5. **PoseArray** (Particules AMCL)
- **Topic** : `/particlecloud`
- **Rôle** : Affiche les particules de localisation AMCL
- **Visualisation** : Flèches représentant les hypothèses de position du robot
- **Utilité** : Voir la convergence des particules vers la position réelle (mode AMCL uniquement)

#### 6. **Polygon** (Empreinte du robot)
- **Topic** : `/local_costmap/published_footprint`
- **Rôle** : Affiche l'empreinte du robot sur la carte
- **Utilité** : Vérifier que le robot peut passer dans les passages

#### 7. **Pose** (Position estimée)
- **Topic** : `/amcl_pose`
- **Rôle** : Affiche la position estimée du robot par AMCL
- **Visualisation** : Flèche indiquant la position et l'orientation
- **Utilité** : Vérifier la précision de la localisation (mode AMCL uniquement)

#### 8. **Path** (Trajet du robot)
- **Topic** : `/robot_path`
- **Rôle** : Affiche le trajet parcouru par le robot
- **Visualisation** : Ligne orange montrant l'historique des positions
- **Utilité** : Visualiser le chemin emprunté par le robot, utile pour déboguer la navigation
- **Publication** : Nœud `odom_to_path` qui s'abonne à `/odometry/filtered` et publie le Path

### Outils de navigation

#### **SetInitialPose** (Estimation de pose initiale)
- **Topic** : `/initialpose`
- **Rôle** : Permet de définir manuellement la pose initiale du robot sur la carte
- **Utilisation** :
  1. Cliquer sur l'outil "2D Pose Estimate"
  2. Cliquer sur la carte à l'endroit où se trouve le robot
  3. Faire glisser pour définir l'orientation
- **Utilité** : Essentiel en mode AMCL pour initialiser la localisation

#### **SetGoal** (Définir un objectif)
- **Topic** : `/goal_pose`
- **Rôle** : Permet de définir un objectif de navigation pour le robot
- **Utilisation** :
  1. Cliquer sur l'outil "2D Goal Pose"
  2. Cliquer sur la carte à l'endroit où vous voulez que le robot aille
  3. Faire glisser pour définir l'orientation finale
- **Utilité** : Commander le robot pour qu'il se déplace vers un point précis

## Utilisation

### Premiers pas : Lancer RViz2

Une fois RViz2 lancé, vous verrez :

#### Panneau de gauche : Displays
- Liste de tous les éléments affichés (Map, LaserScan, TF, etc.)
- Cochez/décochez pour afficher/masquer un élément
- Cliquez sur un élément pour modifier ses propriétés (couleur, taille, topic, etc.)

#### Panneau central : Vue 3D
- **Navigation** :
  - **Clic gauche + glisser** : Rotation de la vue
  - **Clic molette + glisser** : Translation de la vue
  - **Molette** : Zoom avant/arrière
- **Outils** (barre d'outils en haut) :
  - **Interact** : Mode interactif par défaut
  - **2D Pose Estimate** : Définir la pose initiale du robot
  - **2D Goal Pose** : Définir un objectif de navigation

#### Panneau de droite : Propriétés
- Affiche les propriétés de l'élément sélectionné dans Displays
- Permet de modifier les paramètres (couleurs, topics, etc.)

#### Barre d'outils en bas
- **Time** : Affiche le temps ROS2
- **Frame** : Frame de référence actuelle (généralement `odom` ou `map` selon le mode)

### Workflow complet : Visualiser le robot en action

1. **Démarrer les nœuds ROS2 sur le robot** :
   ```bash
   # Sur le robot
   ssh techlab@192.168.40.101
   cd ~/Documents/ANDRA_2025-2026
   source scripts/setup.sh
   ./scripts/launch.sh slam  # Mode SLAM pour créer une carte
   ```

2. **Lancer RViz2 sur votre PC Linux** :
   ```bash
   # Avec l'image Docker (sur une machine Linux du TechLab)
   cd ros-docker
   ./launch.sh  # Lance le conteneur Docker avec --net=host
   # Dans le conteneur :
   cd /workspace
   rviz2 -d ros_launcher/config.rviz
   ```

   **Note importante** : Le conteneur Docker utilise `--net=host` pour communiquer avec le robot. Assurez-vous que :
   - Votre machine Linux est sur le même réseau WiFi que le robot
   - `ROS_DOMAIN_ID=0` est défini (défini automatiquement par `scripts/setup.sh` sur le robot)
   - Le middleware DDS est cohérent (FastRTPS par défaut, configuré dans `scripts/setup.sh`)

3. **Observer la visualisation** :
   - La carte se construit progressivement (mode SLAM)
   - Les scans LIDAR apparaissent en temps réel
   - Le robot est représenté par une flèche sur la carte
   - Le trajet du robot est visualisé via le Path `/robot_path` (ligne orange)

4. **Interagir avec le robot** :
   - Utiliser "2D Goal Pose" pour envoyer le robot à un endroit précis
   - Utiliser "2D Pose Estimate" pour corriger la position estimée (mode AMCL)

### Sans fichier de configuration

Si vous préférez configurer RViz2 manuellement :

```bash
# Lancer RViz2 sans configuration
rviz2

# Puis ajouter manuellement les displays :
# 1. Cliquez sur "Add" (en bas à gauche)
# 2. Sélectionnez "Map" → OK → Topic: /map
# 3. Cliquez sur "Add" → "LaserScan" → OK → Topic: /scan
# 4. Cliquez sur "Add" → "TF" → OK
# 5. Répétez pour les autres displays nécessaires
```

**Conseil** : Une fois configuré, sauvegardez votre configuration via **File → Save Config As** pour la réutiliser plus tard.

## Interprétation des cartes SLAM

### Construction de la carte en temps réel

En mode **SLAM**, vous verrez la carte se construire progressivement :

1. **Démarrage** : La carte est vide (tout en gris)
2. **Déplacement du robot** : Les zones explorées deviennent blanches (libres) ou noires (obstacles)
3. **Convergence** : Plus le robot explore, plus la carte devient précise

### Indicateurs de qualité

- **Carte cohérente** : Les murs sont droits, les angles sont nets
- **Carte floue** : Problème d'odométrie ou de vibrations
- **Carte déformée** : Erreurs d'intégration des capteurs

### Sauvegarde de la carte

Une fois la carte construite, vous pouvez la sauvegarder :

```bash
# Sur le robot
cd ~/Documents/ANDRA_2025-2026/ros_launcher
ros2 run nav2_map_server map_saver -f map_results/ma_carte
```

Cela créera deux fichiers :
- `ma_carte.yaml` : Métadonnées de la carte
- `ma_carte.pgm` : Image de la carte (noir et blanc)

## Visualisation des données des capteurs

### LIDAR (`/scan`)

- **Points visibles** : Le LIDAR fonctionne correctement
- **Aucun point** : Vérifier la connexion et la configuration du LIDAR
- **Points étranges** : Vérifier l'orientation du LIDAR (transformation TF)

### Odométrie (`/odom`)

- **TF `odom → base_link`** : Vérifier que le robot se déplace correctement
- **Dérive** : Erreurs d'intégration des capteurs (normal sur de longues distances)

### Caméra ZED2

- **TF `base_link → zed_camera_link`** : Vérifier la position de la caméra
- **Images** : Utiliser d'autres outils pour visualiser les images (rqt_image_view)


## Notes techniques

### Configuration Middleware DDS

**Qu'est-ce que le middleware DDS ?**

Le middleware DDS (Data Distribution Service) est la couche de communication sous-jacente utilisée par ROS2 pour permettre aux nœuds de communiquer entre eux. C'est lui qui gère :
- La publication et la souscription aux topics ROS2
- La découverte automatique des nœuds sur le réseau
- La transmission des messages entre les nœuds
- La gestion de la qualité de service (QoS)

**Configuration automatique par `scripts/setup.sh`**

Le script `scripts/setup.sh` configure automatiquement :
- **`ROS_DOMAIN_ID=0`** : Identifiant du domaine ROS2 (doit être identique sur tous les nœuds qui doivent communiquer)
- **`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`** : Middleware DDS utilisé (FastRTPS)

**Important pour la communication réseau** :
- Pour que RViz2 dans le conteneur Docker puisse voir les topics du robot, les deux doivent avoir :
  - Le même `ROS_DOMAIN_ID` (généralement `0`)
  - Le même middleware DDS (`rmw_fastrtps_cpp` pour FastRTPS)
- Si les middlewares diffèrent (FastRTPS vs CycloneDDS), les topics ne seront pas visibles même si le réseau fonctionne

**Vérification de la configuration** :
```bash
# Vérifier ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Doit afficher 0

# Vérifier le middleware DDS
echo $RMW_IMPLEMENTATION  # Doit afficher rmw_fastrtps_cpp
```

## Liens utiles

- [Documentation RViz2](https://github.com/ros2/rviz)
- [Documentation SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Documentation Nav2](https://navigation.ros.org/)
- [Documentation FastRTPS](https://fast-dds.docs.eprosima.com/)

