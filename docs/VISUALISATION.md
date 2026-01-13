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

```bash
# Sur le robot, lancer RViz2 avec la configuration
cd ~/Documents/ANDRA_2025-2026
rviz2 -d ros_launcher/config.rviz
```

### Sans fichier de configuration

Si vous préférez configurer RViz2 manuellement :

```bash
# Lancer RViz2 sans configuration
rviz2

# Puis ajouter manuellement les displays :
# 1. Add → Map → Topic: /map
# 2. Add → LaserScan → Topic: /scan
# 3. Add → TF
# etc.
```

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

## Personnalisation de la configuration

Vous pouvez modifier `ros_launcher/config.rviz` pour :
- Changer les couleurs des displays
- Ajouter/supprimer des displays
- Modifier les topics
- Ajuster les paramètres de visualisation

**Note** : Sauvegardez une copie avant de modifier si vous voulez revenir à la configuration par défaut.

## Liens utiles

- [Documentation RViz2](https://github.com/ros2/rviz)
- [Documentation SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Documentation Nav2](https://navigation.ros.org/)
