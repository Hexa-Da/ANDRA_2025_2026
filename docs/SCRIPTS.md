## Scripts disponibles

### `scripts/setup.sh` - Initialisation de l'environnement

Sourcer tous les workspaces nécessaires :

```bash
source scripts/setup.sh
```

Ce script :
- ✅ Initialise ROS2 Humble
- ✅ Source le workspace YDLidar
- ✅ Source le workspace scout_base 
- ✅ Source le workspace ZED Wrapper 
- ✅ Source le workspace ANDRA
- ✅ Configure automatiquement le middleware DDS

### `scripts/build.sh` - Compilation des workspaces

Compiler les workspaces :

```bash
# Compiler tout
./scripts/build.sh 

# Compiler uniquement YDLidar
./scripts/build.sh ydlidar

# Compiler uniquement scout_base
./scripts/build.sh scout_base

# Compiler uniquement ZED Wrapper
./scripts/build.sh zed

# Compiler uniquement ros2_ws
./scripts/build.sh ros2_ws
```

### `scripts/launch.sh` - Lancement du système

Lancer le système complet :

**Mode SLAM** (pour créer une carte) :
```bash
./scripts/launch.sh slam

# Lancer sans LIDAR
./scripts/launch.sh slam enable_lidar:=false

# Lancer sans Scout Base 
./scripts/launch.sh slam enable_scout:=false

# Lancer sans ZEDs
./scripts/launch.sh slam enable_zed:=false

# Lancer sans caméra PTZ
./scripts/launch.sh slam enable_ptz:=false

# Combinaisons possibles
./scripts/launch.sh slam enable_lidar:=false enable_scout:=false
```

**Mode AMCL** (pour utiliser une carte existante) :
```bash
./scripts/launch.sh amcl ros_launcher/andra.yaml
```

### `scripts/ptz-network-setup.sh` - Configuration réseau PTZ

Configure automatiquement l'interface Ethernet pour accéder à la caméra PTZ :

```bash
# Exécuter le script (nécessite les droits sudo)
sudo bash scripts/ptz-network-setup.sh
```

Ce script :
- Configure l'adresse IP statique `192.168.5.100/24` sur `enP8p1s0`
- Supprime les routes WiFi conflictuelles vers `192.168.5.0/24`
- Vérifie que la route passe bien par Ethernet

**Note** : À exécuter après chaque redémarrage du robot pour accéder à la caméra PTZ.

### `video_launcher/script.sh` - Enregistrement vidéo depuis la caméra PTZ

Script interactif pour enregistrer des vidéos depuis le flux RTSP de la caméra PTZ avec retouches d'image :

```bash
cd ~/Documents/ANDRA_2025-2026
./video_launcher/script.sh
```

**Contrôles** :
- `r` : Démarrer/arrêter l'enregistrement (toggle)
- `s` : Arrêter l'enregistrement en cours
- `q` : Quitter le script (arrête aussi l'enregistrement en cours)
