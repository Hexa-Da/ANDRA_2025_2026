# Guide de contrôle PTZ et création de presets

Ce guide explique comment contrôler la caméra PTZ Marshall CV-605 et créer des presets VISCA pour des positions reproductibles.

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

### Méthode 2 : Retour à la position centrale (Home)

Pour revenir rapidement à la position centrale :

```bash
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: -1}"
```

### Méthode 3 : Contrôle continu avec un script Python

Pour un contrôle plus fluide, créez un petit script :

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PTZManualControl(Node):
    def __init__(self):
        super().__init__('ptz_manual_control')
        self.pub = self.create_publisher(Twist, '/ptz/cmd_vel', 10)
        time.sleep(1.0)  # Attendre que le publisher soit prêt
    
    def move(self, pan, tilt, duration):
        """Déplace la PTZ pendant une durée donnée"""
        twist = Twist()
        twist.angular.z = float(pan)
        twist.linear.y = float(tilt)
        
        start = time.time()
        rate = 10  # Hz
        period = 1.0 / rate
        
        while time.time() - start < duration:
            self.pub.publish(twist)
            time.sleep(period)
        
        # Arrêter
        twist.angular.z = 0.0
        twist.linear.y = 0.0
        self.pub.publish(twist)

if __name__ == '__main__':
    rclpy.init()
    controller = PTZManualControl()
    
    # Exemple : tourner à gauche pendant 5 secondes
    controller.move(-1.0, 0.0, 5.0)
    
    rclpy.shutdown()
```

---

## Création de presets

Les presets VISCA permettent d'enregistrer une position précise de la caméra et de la rappeler instantanément. Les presets sont stockés dans la mémoire de la caméra.

### Méthode 1 : Création manuelle avec commande VISCA directe

Pour créer un preset, vous devez :
1. Positionner la caméra à la position désirée (via `/ptz/cmd_vel`)
2. Envoyer la commande VISCA d'enregistrement

**Script Python pour créer un preset** :

```python
#!/usr/bin/env python3
"""
Script pour créer un preset PTZ manuellement.

Usage:
    python3 create_preset.py <numéro_preset> <description>

Exemple:
    python3 create_preset.py 1 "90° à gauche"
"""

import sys
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def save_preset_visca(preset_num, cam_ip='192.168.5.163', cam_port=1259):
    """Enregistre un preset directement via VISCA"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((cam_ip, cam_port))
        
        # Commande VISCA pour enregistrer un preset: 8x 01 04 3F 01 pq FF
        # x = adresse caméra (1), pq = numéro preset (0-127)
        header = bytes([0x80 + 1])  # Adresse caméra = 1
        command = bytes([0x01, 0x04, 0x3F, 0x01, preset_num])
        terminator = bytes([0xFF])
        packet = header + command + terminator
        
        sock.send(packet)
        sock.close()
        
        print(f"✅ Preset {preset_num} enregistré avec succès")
        return True
    except Exception as e:
        print(f"❌ Erreur lors de l'enregistrement du preset: {e}")
        return False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 create_preset.py <numéro_preset> [description]")
        print("Exemple: python3 create_preset.py 1 '90° à gauche'")
        sys.exit(1)
    
    preset_num = int(sys.argv[1])
    
    if not (0 <= preset_num <= 127):
        print("❌ Le numéro de preset doit être entre 0 et 127")
        sys.exit(1)
    
    description = sys.argv[2] if len(sys.argv) > 2 else f"Preset {preset_num}"
    
    print(f"Enregistrement du preset {preset_num}: {description}")
    print("⚠️  Assurez-vous que la caméra est positionnée à la position désirée!")
    input("Appuyez sur Entrée pour continuer...")
    
    if save_preset_visca(preset_num):
        print(f"Preset {preset_num} ({description}) enregistré avec succès!")
    else:
        print("Échec de l'enregistrement")
        sys.exit(1)
```

**Utilisation** :

```bash
# 1. Positionner la caméra manuellement
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
# ... attendre que la caméra atteigne 90° à gauche ...

# 2. Enregistrer la position comme preset 1
python3 create_preset.py 1 "90° à gauche"
```

### Méthode 2 : Utilisation du script setup_ptz_presets.py (modifié)

Le script `setup_ptz_presets.py` peut être modifié pour utiliser directement les commandes VISCA. Voici comment l'adapter :

```python
# Dans setup_ptz_presets.py, remplacer save_preset() par :

def save_preset_visca(self, preset_num):
    """Enregistre un preset directement via VISCA"""
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect(('192.168.5.163', 1259))
        
        header = bytes([0x80 + 1])
        command = bytes([0x01, 0x04, 0x3F, 0x01, preset_num])
        terminator = bytes([0xFF])
        packet = header + command + terminator
        
        sock.send(packet)
        sock.close()
        return True
    except Exception as e:
        self.get_logger().error(f"Erreur VISCA: {e}")
        return False
```

### Presets recommandés

Pour le projet ANDRA, les presets suivants sont recommandés :

- **Preset 0** : Position centrale (Home)
- **Preset 1** : 90° à gauche
- **Preset 2** : Vers le haut
- **Preset 3** : 90° à droite

**Procédure de configuration** :

1. Lancer le système ROS2 :
   ```bash
   source scripts/setup.sh
   ./scripts/launch.sh slam
   ```

2. Positionner et enregistrer chaque preset :
   ```bash
   # Preset 0 : Centre
   ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: -1}"  # Home
   python3 create_preset.py 0 "Position centrale"
   
   # Preset 1 : 90° gauche
   # ... positionner manuellement à 90° gauche ...
   python3 create_preset.py 1 "90° à gauche"
   
   # Preset 2 : Vers le haut
   # ... positionner manuellement vers le haut ...
   python3 create_preset.py 2 "Vers le haut"
   
   # Preset 3 : 90° droite
   # ... positionner manuellement à 90° droite ...
   python3 create_preset.py 3 "90° à droite"
   ```

---

## Utilisation des presets

### Appeler un preset existant

Une fois les presets créés, vous pouvez les appeler via le topic `/ptz/preset` :

```bash
# Appeler le preset 0 (centre)
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 0}"

# Appeler le preset 1 (90° gauche)
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 1}"

# Appeler le preset 2 (vers le haut)
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 2}"

# Appeler le preset 3 (90° droite)
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 3}"

# Retour au centre (Home)
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: -1}"
```

### Utilisation dans sequence_robot

Le script `sequence_robot` utilise automatiquement les presets si `use_presets = True` :

```bash
ros2 run navigation_utils sequence_robot
```

La séquence utilise :
- **Preset 1** à 0s → capture à 5s
- **Preset 0** à 10s → **Preset 2** à 12.5s → capture à 15s
- **Preset 0** à 20s → **Preset 3** à 22.5s → capture à 25s
- **Preset 0** à 30s → nouvelle séquence

---

## Exemples pratiques

### Exemple 1 : Balayage horizontal complet

```bash
# Aller à gauche
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 1}"
sleep 3

# Retour au centre
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 0}"
sleep 3

# Aller à droite
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 3}"
sleep 3

# Retour au centre
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 0}"
```

### Exemple 2 : Vérifier les presets

```bash
# Tester chaque preset
for i in 0 1 2 3; do
    echo "Test preset $i"
    ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: $i}"
    sleep 3
done

# Retour au centre
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 0}"
```

### Exemple 3 : Créer un nouveau preset personnalisé

```bash
# 1. Positionner la caméra à la position désirée
ros2 topic pub --once /ptz/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
# Attendre que la caméra atteigne la position...

# 2. Enregistrer comme preset 5
python3 create_preset.py 5 "Position personnalisée haut-gauche"

# 3. Tester le preset
ros2 topic pub --once /ptz/preset std_msgs/Int32 "{data: 5}"
```

---

## Dépannage

### La caméra ne bouge pas

1. Vérifier que `ptz_controller` est lancé :
   ```bash
   ros2 node list | grep ptz_controller
   ```

2. Vérifier les topics :
   ```bash
   ros2 topic list | grep ptz
   ros2 topic echo /ptz/cmd_vel
   ```

3. Vérifier la connexion réseau :
   ```bash
   ping 192.168.5.163
   ```

### Les presets ne fonctionnent pas

1. Vérifier que les presets sont bien enregistrés dans la caméra (via l'interface web de la caméra si disponible)

2. Réessayer l'enregistrement avec le script `create_preset.py`

3. Vérifier que le numéro de preset est valide (0-127)

---

## Références

- **Protocole VISCA** : Documentation Marshall CV-605
- **Topics ROS2** :
  - `/ptz/cmd_vel` : Contrôle par vitesse (Twist)
  - `/ptz/preset` : Appel de presets (Int32)
- **Nœud** : `ptz_controller` dans le package `image_transfer`
