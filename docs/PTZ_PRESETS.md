# Guide de contrôle PTZ

Ce guide explique comment contrôler la caméra PTZ Marshall CV-605 via les commandes de base `/ptz/cmd_vel`. 

## Spécifications Marshall CV605 (manuel CV605-BK/WH)

- **Pan** : ±135° (270° au total)
- **Tilt constructeur** : -30° à +30° (60° au total)
- **Tilt réel** : -90° à +90° (180° au total) 
- **Presets** : 0 & -1 (voir ci-dessous)

## Contrôle manuel de la caméra

### Prérequis

Assurez-vous que le système ROS2 est lancé avec le nœud `ptz_controller` :

```bash
source scripts/setup.sh
./scripts/launch.sh slam
```

### Méthode 1 : Contrôle par vitesse (Twist)

La caméra peut être contrôlée en envoyant des commandes de vitesse sur le topic `/ptz/cmd_vel`.

**Format du message** :
- `angular.z` : Pan (rotation horizontale)
  - Valeur positive = rotation vers la droite
  - Valeur négative = rotation vers la gauche
  - Plage recommandée : -1.0 à 1.0
- `linear.y` : Tilt (rotation verticale)
  - Valeur positive = rotation vers le haut
  - Valeur négative = rotation vers le bas
  - Plage recommandée : -1.0 à 1.0

**Exemples de commandes** :

```bash
# Tourner à gauche (pan négatif)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"

# Tourner à droite (pan positif)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Tourner vers le haut (tilt positif)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Tourner vers le bas (tilt négatif)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Mouvement diagonal (haut-droite)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Arrêter le mouvement (toutes les valeurs à 0)
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Méthode 2 : Preset

Pour revenir rapidement à la position centrale ou recalibrer la position :

```bash
# Retour au centre avec preset Home
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: -1}"

# Reset et calibrage des moteurs
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 0}"
```

### Utilisation dans sequence_photo

Le script `sequence_photo` exécute un cycle "Step-and-Go" : le robot avance d'un mètre, s'immobilise, puis réalise la capture. À chaque arrêt, la caméra est pilotée via `/ptz/cmd_vel` et le preset Home (preset -1).

```bash
ros2 run navigation_utils sequence_photo
```

Voici la sequence la plus rapide efficace qu'il est possible de faire d'apres nos tests :

1. **Gauche** :
   - Mouvement pan gauche (3.6s) → Stabilisation (0.5s) → Capture → Attendre (0.5s)

2. **Haut #1** :
   - Mouvement tilt haut (3.0s) → Stabilisation (0.5s) → Capture → Attendre (0.5s)

3. **Haut #2** :
   - Mouvement tilt haut (4.0s) → Stabilisation (0.5s) → Capture → Attendre (0.5s) → Retour Home (4.5s)

4. **Droite** :
   - Mouvement pan droite (3.6s) → Stabilisation (0.5s) → Capture → Attendre (0.5s)

5. **Haut #3** :
   - Mouvement tilt haut (3.0s) → Stabilisation (0.5s) → Capture → Attendre (0.5s) → Retour Home (1.5s)

**Paramètres configurables** :
- `ptz_move_90_duration:=3.6` : Durée du mouvement pan (gauche/droite) en secondes
- `ptz_move_up_short_duration:=3.0` : Durée du mouvement tilt court (haut) en secondes
- `ptz_move_up_long_duration:=4.0` : Durée du mouvement tilt long (haut) en secondes
- `ptz_stable_duration:=0.5` : Durée de stabilisation avant capture en secondes
- `delay_after_capture:=0.5` : Attente après capture avant mouvement suivant en secondes
- `ptz_home_duration_mid:=4.5` : Durée pour retour home après capture 3 (haut #2) en secondes
- `ptz_home_duration_final:=1.5` : Durée pour retour home final après capture 5 (haut #3) en secondes

**Arrêt (Ctrl+C)** : Le script envoie STOP puis HOME directement via socket VISCA, indépendant de ROS2, pour garantir le retour au centre même si le contexte ROS2 est invalidé.

---

### Utilisation dans sequence_video

Le script `sequence_video` automatise l'acquisition de données pour l'inspection des parois. Il synchronise l'avance lente du robot avec un balayage PTZ optimisé pour la détection.

```bash
ros2 run navigation_utils sequence_video

```

**Pattern "Scanner de Voûte" (Recommandé pour l'IA)** :

Ce pattern utilise un **balayage sinusoïdal horizontal**. Il permet de transformer la caméra en un scanner de ligne haute qui "déroule" la surface du tunnel pendant que le robot avance.

1. **Axe Pan (Horizontal)** : Oscillation continue entre l'extrême gauche (`-1.0`) et l'extrême droite 
(`1.0`). La vitesse suit une courbe en cosinus : elle ralentit aux extrémités pour stabiliser l'image 
avant l'inversion du mouvement.
2. **Axe Tilt (Vertical)** : Fixe pendant le balayage. Une légère poussée (0.1) est appliquée uniquement au début du premier cycle pour positionner vers la voûte.
3. **Avance Robot** : Le robot maintient une vitesse constante (`0.06 m/s`). Chaque passage 
horizontal de la caméra couvre une nouvelle section de tunnel avec un léger recouvrement (*overlap*) 
nécessaire à la reconstruction post-traitement.

**Arrêt (Ctrl+C)** : Le script envoie STOP puis HOME directement via socket VISCA, indépendant de ROS2, pour garantir le retour au centre même si le contexte ROS2 est invalidé.

**Paramètres configurables** :

| Paramètre | Valeur par défaut | Description |
| --- | --- | --- |
| `robot_speed` | `0.06` | Vitesse d'avance du robot (m/s) |
| `ptz_sweep_duration` | `30.0` | Temps pour un aller-retour horizontal complet |
| `ptz_start_pan` | `-1.0` | Limite gauche du balayage |
| `ptz_end_pan` | `0.0` | Limite droite du balayage (centre) |
| `ptz_start_tilt` | `0.0` | Tilt de départ |
| `ptz_end_tilt` | `1.0` | Tilt cible (poussée au 1er cycle uniquement) |
| `ptz_speed_factor` | `10.0` | Gain appliqué aux commandes de vitesse PTZ |
| `rtsp_url` | `rtsp://...` | Flux source de la Marshall CV605 |
| `enable_video_adjustment` | `false` | Activer le traitement logiciel (Luminosité/Contraste) |