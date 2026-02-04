# Guide de contrôle PTZ

Ce guide explique comment contrôler la caméra PTZ Marshall CV-605 via les commandes de base `/ptz/cmd_vel`. 

## Spécifications Marshall CV605 (manuel CV605-BK/WH)

- **Pan** : ±135° (270° au total)
- **Tilt** : -30° à +30° (60° au total)
- **Presets** : 0–9 sur la télécommande, jusqu’à 255 en VISCA

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

**Note** : Le script `sequence_robot` utilise automatiquement le preset Home (-1) pour revenir au centre après chaque capture. Le nœud `ptz_controller` gère ce topic et envoie la commande VISCA appropriée à la caméra.


**Important** : Le topic `/ptz/preset` n'est plus géré automatiquement par le système. Si vous voulez utiliser des presets, vous devez créer un nœud personnalisé qui écoute ce topic et envoie les commandes VISCA appropriées à la caméra.

### Utilisation dans sequence_robot

Le script `sequence_robot` utilise **uniquement les commandes de base `/ptz/cmd_vel`** et le preset Home (-1) pour contrôler la PTZ car beaucoup plus simple a configurer :

```bash
ros2 run navigation_utils sequence_robot
```

La séquence suit une séquence précise avec durées fixes :

1. **Gauche** :
   - Mouvement pan gauche (3.5s) → Stabilisation (0.5s) → Capture → Attendre (0.5s) → Retour Home (3.5s)

2. **Haut** :
   - Mouvement tilt haut (5.5s) → Stabilisation (0.5s) → Capture → Attendre (0.5s) → Retour Home (5.5s)

3. **Droite** :
   - Mouvement pan droite (3.5s) → Stabilisation (0.5s) → Capture → Attendre (0.5s) → Retour Home (3.5s)

**Paramètres configurables** :
- `ptz_move_90_duration:=3.5` : Durée du mouvement pan (gauche/droite) en secondes
- `ptz_move_up_duration:=5.5` : Durée du mouvement tilt (haut) en secondes
- `ptz_stable_duration:=0.5` : Durée de stabilisation avant capture en secondes
- `delay_after_capture:=0.5` : Attente après capture avant retour home en secondes
- `ptz_center_duration_pan:=3.5` : Durée pour retour home depuis pan (gauche/droite) en secondes
- `ptz_center_duration_tilt:=5.5` : Durée pour retour home depuis tilt (haut) en secondes

---
