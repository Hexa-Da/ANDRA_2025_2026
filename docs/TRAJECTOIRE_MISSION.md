# Guide mission trajectoire (AMCL + Nav2)

Ce document décrit la création et l'utilisation du nœud `trajectoire_mission` pour exécuter une mission autonome à partir de waypoints sur carte.

## 1) Prérequis

- Carte existante (`.yaml` + image) pour AMCL
- Stack navigation lancée en mode AMCL avec Nav2 actif
- Package `navigation_utils` compilé et sourcé

Exemple de lancement:

```bash
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
./scripts/launch.sh amcl ros2_ws/src/ros_launcher/map_results/ma_carte.yaml laser_mount_yaw:=0.0
```

## 2) Nœud `trajectoire_mission`

Fichier: `ros2_ws/src/navigation_utils/navigation_utils/trajectoire_mission.py`

Rôle:

- lit une liste de waypoints (`x`, `y`, `yaw`) depuis un fichier YAML
- envoie les objectifs à Nav2 (`navigate_to_pose`) dans le repère `map`
- exécute les points séquentiellement
- peut fonctionner en boucle (`loop:=true`)
- intègre un arrêt d'urgence frontal LiDAR:
  - surveillance `/scan`
  - annulation du goal courant si obstacle frontal sous un seuil

## 3) Format des fichiers trajectoire

Dossier recommandé:

- `ros2_ws/src/navigation_utils/trajectoire/`

Format YAML attendu:

```yaml
waypoints:
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - {x: 2.5, y: -0.4, yaw: 1.57}
```

Notes:

- `yaw` est en radians
- repère utilisé: `map`
- pour une boucle fermée explicite, répéter le point de départ en fin de liste

## 4) Créer un `traj.yaml` depuis RViz sans déplacer le robot (activer le switch)

1. Lancer AMCL + Nav2, fixer `Fixed Frame = map` dans RViz
2. Dans un terminal, lancer:

```bash
ros2 topic echo /goal_pose
```

3. Dans RViz, cliquer les points avec `2D Nav Goal`
4. Pour chaque point, relever:
   - `position.x`
   - `position.y`
   - `orientation.z`
   - `orientation.w`
5. Convertir en yaw:

```text
yaw = 2 * atan2(z, w)
```

6. Reporter les points dans le fichier `traj.yaml`

## 5) Lancer une mission

```bash
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
ros2 run navigation_utils trajectoire_mission --ros-args \
  -p waypoints_file:=/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/src/navigation_utils/trajectoire/ma_carte_traj.yaml \
  -p frame_id:=map \
  -p loop:=false \
```

Boucle continue:

```bash
ros2 run navigation_utils trajectoire_mission --ros-args \
  -p waypoints_file:=/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/src/navigation_utils/trajectoire/ma_carte_traj.yaml \
  -p frame_id:=map \
  -p loop:=true \
```
