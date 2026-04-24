# TODO - Projet ANDRA 2025-2026

## Contexte du projet

Projet réalisé par un groupe de 4 étudiants en partenariat avec l'ANDRA.

Mission : rendre le robot Agilex Scout Mini autonome dans les galeries de l'ANDRA pour effectuer une analyse des fissures sur les murs via une caméra 3D.

**Transmission d'année en année** : Ce projet est transmis d'année en année parmi les étudiants. Le but est de s'approprier les avancées réalisées l'année dernière pour reproduire leurs résultats finaux, puis réaliser les missions supplémentaires demandées par l'ANDRA.

## Échéances importantes

- **Soutenance mi-parcours** : 23 janvier 2026
- **Première descente** : 6 février 2026
- **Seconde descente** : 10 avril 2026
- **Soutenance finale** : 12 juin 2026

---

## Ce qui a été fait

### Accès et ressources
- [x] Accès au dépôt GitHub
- [x] Accès au drive hébergeant le modèle YOLO11
- [x] Accès au robot Agilex Scout Mini du TechLab

### Infrastructure et scripts
- [x] Analyse et compréhension de l'infrastructure et des scripts
- [x] Établir un workflow simple pour toute l'équipe
- [x] Création d'un script d'initialisation (`scripts/setup.sh`)
- [x] Création d'un script de compilation (`scripts/build.sh`)
- [x] Création d'un script de lancement (`scripts/launch.sh`)
- [x] Création d'un script de mise en réseau de la PTZ (`scripts/ptz-network-setup.sh`)
- [x] Réécriture du script vidéo, avec sauvegarde dans `video_output/` (`video/script.sh`)
- [x] Organisation de la structure du projet (workspaces, dépendances, configurations/launcher)
- [x] Nettoyage du code, fichiers non utilisés ou dépréciés

### Documentation
- [x] Documentation complète sur comment utiliser le robot (`DEMARRAGE_ROBOT.md`)
- [x] Guide connexion réseau TechLab vs Hotspot (`docs/HOTSPOT.md`)
- [x] Explication de la structure du projet (`STRUCTURE.md`)
- [x] Guide d'utilisation des scripts (`SCRIPTS.md`)
- [x] Guide de débogage (`DEBUG.md`)
- [x] Guide de contrôle PTZ (`PTZ_PRESETS.md`)
- [x] Guide de visualisation RViz2 (`VISUALISATION.md`)
- [x] Documentation de l'arbre TF (`TF_TREE.md`)
- [x] Documentation détection YOLO et TensorRT (`docs/DETECTION_YOLO.md`)
- [x] Guide mission trajectoire AMCL/Nav2 (`docs/TRAJECTOIRE_MISSION.md`)

### Vérification/Nettoyage des nœuds ROS2 présents
- [x] `image_publisher` : Capture des images depuis la caméra PTZ, sauvegarde dans `images_capturees/`
- [x] `image_subscriber` : Détection YOLO des fissures, sauvegarde dans `images_detectees/`
- [ ] `position_publisher` : Publie la position courante du robot (extraite depuis `/odometry/filtered`)
- [ ] `show_pos` : Affichage de la position du robot
- [ ] `report_fissures` : Tracé des positions détectées sur la carte
- [x] `ptz_controller` : Contrôle PTZ de la caméra Marshall CV-605 via protocole VISCA over IP

### Implémentation de nœuds ROS2
- [x] `sequence_photo` : Automatisation de la séquence de mouvement et captures PTZ en boucle (5 captures)
- [x] `sequence_video` : Enregistrement vidéo continu avec balayage PTZ automatique
- [x] `video_publisher` : Traitement des vidéos enregistrées, extraction d'images et publication sur `/photo_topic`
- [x] `odom_to_path` : Transforme `/odom` en `/robot_path` pour la visualisation.

### Configuration navigation
- [x] Configuration SLAM Toolbox (`ros_launcher/configs/slam_config.yaml`)
- [x] Configuration AMCL (`ros_launcher/configs/amcl_config.yaml`)
- [x] Configuration EKF pour fusion des capteurs (`ros_launcher/configs/ekf_config.yaml`)
- [x] Configuration LIDAR (`ros_launcher/configs/ydlidar_TG15.yaml` — fichier custom, paramètres adaptés au TG15)
- [x] Launch file principal (`ros_launcher/navigation_stack.launch.py`)
- [x] Transforms TF statiques révisés

### Utilisation des capteurs
- [x] LiDAR YDLidar TG15 : en service ; scan OK sur `/scan` ; USB `/dev/ttyUSB0`, baud 512000, 10 Hz, 20 kHz ; problèmes hardware (Micro USB, mauvaise config G4 vs TG15) résolus ; fichier custom `ydlidar_TG15.yaml` + paramètre `laser_mount_yaw` selon montage (π/2 vs 0 pour SLAM) ; qualité de carte dégradée par les piliers (support 3D pour carto).
- [x] Caméra ZED 2i : driver `zed_wrapper` installé et OK ; odom/IMU/image publiés ; EKF : IMU ZED pour `vyaw` seulement, pas de fusion de la VO `/zed/zed_node/odom` ; profondeur/nuages désactivés côté GPU pour libérer la charge (YOLO).
- [x] Base Scout / odométrie des roues : `scout_base` fonctionnel via CAN `agilex`, topic `/odom_robot` aligné EKF ; dérive mentionnée roue avant gauche.
- [x] Caméra PTZ Marshall CV-605 : réseau `192.168.5.x` (Jetson en `192.168.5.100`), capture RTSP + pub `/photo_topic`, contrôle VISCA (`ptz_controller`) ; script `ptz-network-setup.sh` pour éviter conflits WiFi.

### Utilisation de RViz
- [x] macOS / Docker : tentative RViz2 via Docker + XQuartz (`launch-macos.sh`) puis abandon (trop complexe) — usage ciblé Linux (machine TechLab).
- [x] DDS dans Docker : `ros2 topic list` / `node list` vides dans le conteneur alors que le robot tournait — diagnostic réseau + `ROS_DOMAIN_ID` OK ; alignement middleware : `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` partout (robot + conteneur).
- [x] État : FastRTPS (`rmw_fastrtps_cpp`) standardisé côté robot et Docker pour une découverte ROS2 cohérente avec RViz lancé depuis le conteneur.

### Configuration Hotspot
- [x] Objectif : en terrain (tunnels ANDRA), sans Wi‑Fi, joindre la Jetson via le hotspot qu’elle crée.
- [x] Usage : SSID `JetsonWIFI`, mot de passe documenté ; SSH préféré en `ssh techlab@orin2.local`.
- [x] Terrain : hotspot testé opérationnel en première descente ; stabilité SSH confirmée au fil des sorties.

### Création de script d'analyse autonome
- [x] `sequence_photo` : boucle autonome — avance du robot (~1 m), arrêt, puis 5 captures PTZ (positions pan/tilt prédéfinies : gauche, haut ×3, droite, haut) via `/ptz/cmd_vel` et déclenchement des clichés (`/trigger_capture`) ; retour au centre (preset Home) ; paramètres (durées, stabilisation) ajustables.
- [x] `sequence_video` : enregistrement vidéo continu en parallèle d’un balayage PTZ (mouvement horizontal type sinusoïde) et du robot à faible vitesse linéaire ; arrêt propre (STOP puis Home).

### Optimisation de l'utilisation GPU
- [x] Docker / Jetson Orin : image dédiée (`Dockerfile.jetson`) et scripts `image_subscriber_gpu.sh`, `launch_jetson.sh` pour exécuter la détection YOLO avec accès GPU.
- [x] TensorRT : conversion du modèle PyTorch vers `best.engine` via `scripts/convert_to_tensorrt.sh` pour accélérer l’inférence (souvent plusieurs fois plus rapide qu’en PyTorch seul).
- [x] `image_subscriber` : charge automatiquement le moteur TensorRT s’il est disponible, avec repli sur `best.pt` sinon.
- [x] ZED : désactivation de la profondeur et des nuages de points côté nœuds ZED pour libérer la mémoire GPU au profit de l’analyse YOLO sur images et vidéos.

### Création des premières cartes
- [x] Contexte : les premiers essais de cartographie échouaient en partie à cause de scans LiDAR peu exploitables (capteur entre les quatre piliers de la tourelle, interférences, configuration à affiner).
- [x] Fichier `ros_launcher/configs/ydlidar_TG15.yaml` : configuration dédiée au TG15 pour améliorer la qualité des scans.
- [x] Plateforme imprimée en 3D : élève le LiDAR pour la cartographie ; possibilité de remonter le capteur entre les piliers pour la navigation et la détection d’obstacles (deux modes d’usage).
- [x] Mise à jour EKF + SLAM : fusion recentrée sur `/odom_robot`, IMU ZED limitée à `vyaw`, suppression de `odom1`, réglages EKF à 15 Hz avec `history_length=10`, et frames SLAM strictes `odom/base_link/map` pour stabiliser la trajectoire et fiabiliser la cartographie.
- [x] Paramètre `laser_mount_yaw` : π/2 (rad) pour le montage courant ; `laser_mount_yaw:=0.0` pour la création de carte en SLAM, selon l’orientation du LiDAR sur la tourelle.
- [x] Résultat : qualité des nuages de points nettement meilleure ; premières cartes exploitables produites, ouvrant la voie à la localisation (AMCL) et aux tests de navigation sur carte.

### Navigation autonome de base
- [x] Réussir à faire avancer le robot de manière autonome sur carte (mission par waypoints)
- [x] Développer la navigation autonome au-delà de "avancer en ligne droite" (séquence de goals Nav2)
- [x] Implémenter la détection et évitement d'obstacles si nous continuons avec le LIDAR
- [x] Créer un nœud de mission trajectoire (`navigation_utils/trajectoire_mission.py`)
- [x] Créer et utiliser des fichiers de trajectoire YAML par carte (`ros2_ws/src/navigation_utils/trajectoire/*.yaml`)

---

## Problèmes rencontrés / résolus

### Réinstallation complète du robot (pas de backup)
- [x] **Problème** : L'image qui avait été créée sur le robot l'année dernière a été supprimée sans sauvegarde
- [x] **Action** : Refaire toute la réinstallation et sourçage des drivers ainsi que leur configuration
- [x] **Objectif** : Atteindre les résultats finaux de l'année dernière

### Problème LIDAR (impossible de faire un scan)
- [x] **Connexion au port série** (`/dev/ttyTHS1`)
- [x] **Diagnostic communication LiDAR** :
  - Scan de baudrates effectués : 115200, 128000, 230400 sans réponse
  - Tentative activation forcée via DTR/RTS sans succès
  - LED s'allume brièvement puis s'éteint (mise en sécurité ou coupure alimentation)
- [x] **Diagnostic du chipset (Carte Radar_Con)** :
  - Le port Micro USB est arraché, ce qui empêche l’alimentation du moteur et donc le lancement du scan
  - Test de loopback concluant (TX/RX opérationnels), le reste des composants fonctionne donc correctement
  - Un branchement de secours via le port USB-C a été tenté et s’est avéré fonctionnel
- [x] **Problème résolu après intervention sur le chipset (après la première descente)** :
  - **Modification de l’alimentation** : Passage au port USB-C `/dev/ttyUSB0` pour fournir suffisamment de puissance au LIDAR.
  - **Correction de la configuration** : Le mauvais fichier de configuration (G4.yaml) était utilisé alors que le LIDAR est en réalité un modèle TG15.
  - **Actions menées** :
    - Remplacement du port `/dev/ttyTHS1` par `/dev/ttyUSB0` via soudure au microscope
    - Suppression de l’ancien fichier de configuration local (`ros_launcher/ydlidar_config.yaml`)
    - Adoption du fichier officiel `TG.yaml` du package `ydlidar_ros2_driver`
    - Simplification du launch file afin d’utiliser directement la configuration officielle fournie par le package
  - **Résultat** : Le LIDAR démarre normalement et publie sur `/scan` avec les paramètres attendus.
- [x] **État actuel** : LIDAR en service
  - Modèle : TG15 (Model Code 100)
  - Port utilisé : `/dev/ttyUSB0` (détection automatique)
  - Baudrate : 512000
  - Fréquence de balayage : 10 Hz
  - Taux d’échantillonnage : 20 kHz
  - Publication sur `/scan` : OK

### Problème ZED 2 (reconfigurer et installer le driver)
- [x] **zed_wrapper** : Package installé et fonctionnel
  - ZED SDK installé dans /usr/local/zed
  - zed_msgs installé via apt
  - zed_wrapper compilé dans dependencies/zed-ros2-wrapper
  - Caméra ZED 2i détectée (S/N 32802052)
  - Topics publiés : /zed/zed_node/odom, /zed/zed_node/imu/data, /zed/zed_node/rgb/color/rect/image
  - IMU ZED toujours utilisée par l'EKF (vyaw) ; la VO `/zed/zed_node/odom` n'est plus fusionnée dans `ekf_config.yaml`
- [x] **Vérification cohérence données** : Cohérence des données renvoyées par la caméra ZED2i au filtre EKF.

### Problèmes scout_base (package et driver manquant + interface CAN non reconnue)
- [x] **Package** : Package non trouvé (nécessaire pour l'odométrie des roues)
  - Repository probable : https://github.com/agilexrobotics/scout_ros2
  - Topic attendu : `/odom_robot`
- [x] **Arrêt du nœud** :
  - **Configuration** : Ajout du paramètre `odom_topic_name:=odom_robot` pour correspondre à la configuration EKF
  - **Problème identifié** : Le driver `ugv_sdk` ne reconnaissait pas "agilex" comme un port CAN valide (vérification stricte du nom contenant "can")
  - **Solution** : Modification du code source `scout_base_ros.cpp` pour accepter "agilex" en plus des noms contenant "can"
  - **État actuel** : Le nœud démarre correctement et communique avec le robot via l'interface CAN `agilex`

### Problème Caméra PTZ (impossible de se connecter)
- [x] **Adresse statique introuvable** : Caméra PTZ inaccessible sur `192.168.5.163`
  - **Cause** : La connexion Ethernet de la Jetson n'était pas sur le même sous-réseau que la caméra PTZ (192.168.5.x)
  - **Solution** : Configuration de l'adresse IP statique sur l'interface Ethernet `enP8p1s0`
  - **Configuration réseau** : Interface configurée avec `192.168.5.100/24` pour accéder à la caméra `192.168.5.163`
- [x] **Conflits** : Le WiFi (`wlP1p1s0`) obtenait aussi l'adresse `192.168.5.100`, créant un conflit de routes
  - **Solution** : Création du script `scripts/ptz-network-setup.sh` qui :
    - Configure automatiquement l'IP sur `enP8p1s0`
    - Supprime les routes WiFi conflictuelles vers `192.168.5.0/24`
    - Vérifie que la route passe bien par Ethernet
  - **État actuel** : La caméra PTZ est accessible, les images sont capturées et publiées sur `/photo_topic`
- [x] **Contrôle PTZ** : opérationnel via VISCA over IP port 1259
  - Caméra : Marshall CV-605 (5x HD60 IP PTZ Camera with 3GSDI)
  - Nœud créé : `ptz_controller` dans le package `image_transfer`
  - Topics :
    - `/ptz/cmd_vel` (geometry_msgs/Twist) : Contrôle pan/tilt continu (utilisé par `sequence_photo` et `sequence_video`)
    - `/ptz/preset` (std_msgs/Int32) : Preset Home (-1) pour retour au centre et Reset (0) pour le recalibrage
  - Format VISCA : Implémentation selon documentation Marshall CV-605
    - Header : `0x80 + camera_address` (adresse 1 par défaut)
    - Pan-Tilt Drive : `0x01 0x06 0x01 VV WW DD DD` où VV=pan speed (1-18), WW=tilt speed (1-14), DD DD=direction
    - Home : `0x01 0x06 0x04` pour retour au centre
  - **État actuel** : Contrôle PTZ fonctionnel, caméra répond aux commandes de mouvement et preset
- [x] **Automatisation séquence robot** : Création des nœuds `sequence_photo` et `sequence_video`
  - sequence_photo : Avance 1m, arrêt 30s, 5 captures PTZ (Gauche, Haut×3, Droite, Haut), boucle
  - sequence_video : Vidéo continue + balayage PTZ horizontal sinusoïdal + robot à 0.06 m/s
  - Contrôle PTZ : `/ptz/cmd_vel` pour les mouvements ; preset Home (-1) ou socket VISCA direct à l'arrêt (Ctrl+C)
  - Paramètres configurables : Durées de mouvement, stabilisation, retour au centre
  - **État actuel** : Séquence automatique fonctionnelle, arrêt propre avec STOP puis HOME via socket

### Améliorations du système de lancement (pas de système de débogage)
- [x] **Options de configuration** : Ajout d'options pour désactiver des composants
  - `enable_lidar:=false` : Désactiver le LIDAR
  - `enable_scout:=false` : Désactiver Scout Base
  - `enable_zed:=false` : Désactiver la caméra ZED
  - `enable_ptz:=false` : Désactiver la caméra PTZ
  - `enable_image_transfer:=false` : Désactiver les nœuds de capture et de traitement d'images
  - Permet de tester le système même si un composant pose problème

### Configuration visualisation RViz2 avec Docker (utilisation de RViz compliquée)
- [x] **Tentative macOS** : Tentative d'utilisation de RViz2 sur macOS via Docker
  - **Solution XQuartz** : Création de `launch-macos.sh` utilisant XQuartz pour l'affichage X11
  - **Décision** : Abandon de la compatibilité macOS/Windows pour simplifier le setup
- [x] **Simplification Linux uniquement** : Retour à un setup Linux uniquement
  - **Raison** : Complexité de configuration trop élevée pour macOS/Windows et machine Linux du TechLab disponible
- [x] **Problème de communication DDS** : Topics et nœuds ROS2 non visibles dans le conteneur Docker
  - **Symptômes** : `ros2 topic list` et `ros2 node list` vides dans le conteneur malgré les nœuds actifs sur le robot
  - **Diagnostics effectués** :
    - Vérification réseau : `ping` fonctionnait entre robot et conteneur
    - Vérification `ROS_DOMAIN_ID` : Configuré à `0` des deux côtés
    - Vérification middleware : Robot utilisait FastRTPS par défaut (port 7400/7401 visibles avec `netstat`)
    - Conteneur utilisait également FastRTPS par défaut
  - **Problème identifié** : Mismatch potentiel de configuration DDS (FastRTPS vs CycloneDDS)
  - **Solution** : Configuration explicite de `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` partout
- [x] **Configuration FastRTPS** : Standardisation sur FastRTPS pour robot et conteneur Docker
  - **État actuel** : Robot et conteneur Docker utilisent maintenant FastRTPS de manière cohérente
- [x] **Amélioration visualisation RViz2** : Ajout de displays et configuration du Fixed Frame
  - **Nœud odom_to_path** : Création de `ros2_ws/src/navigation_utils/navigation_utils/odom_to_path.py`
    - Fonctionnalités :
      - Abonnement à `/odometry/filtered` (odométrie filtrée par EKF)
      - Publication du Path sur `/robot_path` pour visualiser le trajet du robot
      - Publication TF `odom → base_link` pour compléter l'arbre de transformations
    - Paramètres : Ajout d'un point au Path tous les 10 cm minimum (`min_distance = 0.1`)
  - **Configuration RViz2** : Mise à jour de `ros_launcher/config.rviz`
    - Fixed Frame : Changé de `map` à `odom` pour permettre la visualisation même si la carte n'est pas encore créée (mode SLAM initial)
    - Display RobotPath : Ajout d'un display Path pour visualiser `/robot_path` (couleur orange, style Lines)
    - Displays existants : Conservation de tous les displays précédents (Grid, TF, Map, LaserScan, PoseArray, Polygon, Pose)
  - **Frame world** : Ajout de la frame `world` dans l'arbre TF
  - **Reconstruction de l'arbre TF** : Réorganisation de l'arbre des transformations pour intégrer `world` et clarifier les liens entre frames
  - **Conflit avec le nœud odom_to_path** : Le nœud `odom_to_path` publiait aussi `odom → base_link` ; à prendre en compte dans la reconstruction TF (éviter doublon ou conflit avec l'EKF)
  - **Objectif** : Constater la dérive de trajectoire entre les différents capteurs (odométrie roues, ZED, EKF fusionné) pour diagnostiquer le positionnement

### Config Hotspot pour connexion dans les tunnels ANDRA
- [x] **Contexte** : Dans les tunnels, pas de Wi-Fi du TechLab ; le robot doit être joignable via son hotspot (réseau créé par la Jetson).
- [x] **À valider** : Vérifier que le hotspot `JetsonWIFI` (mdp `depinfonancy`) se lance bien quand le robot ne capte pas le TechLab ; connexion SSH via `ssh techlab@orin2.local` (ne pas utiliser `192.168.40.100` en mode hotspot).
- [x] **À tester** : Connexion et stabilité SSH depuis un PC portable connecté au hotspot du robot, dans un environnement extérieur au labo.
- [x] **Référence** : Procédure détaillée dans `docs/HOTSPOT.md` (mode Terrain, commandes `nmcli`).

### Inventaire et analyse hardware
- [x] **Inventaire complet** : Paul-Antoine a effectué un inventaire de l'ensemble des composants matériels du robot et a organisé le câblage interne de la tourelle.
- [x] **Étude de la structure de la tourelle** : Démontage puis remontage de la tourelle pour :
  - Identifier les composants effectivement utilisés ou non
  - Comprendre le rôle et l'utilité de chaque élément
  - Effectuer des tests individuels sur chaque capteur

### Première descente - Vendredi 6 février 2026
- [x] **Réalisation** : Première descente dans les tunnels ANDRA effectuée avec succès
- [x] **Caméra 360** : Caméra 360 prêtée par l'ANDRA, dataset créé
- [x] **Système opérationnel** :
  - Séquence de prise de photo fonctionnelle comme prévu
  - Système de Hotspot opérationnel
  - Tous les nœuds ROS2 fonctionnels
- [x] **Incident matériel** : Alimentation de la Jetson arrachée suite à une mauvaise manipulation
  - **Réparation** : Paul-Antoine a réparé la carte en changeant l'alimentation le week-end suivant
  - **État** : Robot de nouveau opérationnel rapidement
- [x] **Améliorations à réaliser sur la séquence PTZ** :
  - Séquence complète dans une galerie assez longue
  - Séquence actuelle (3 photos) ne couvre pas l'entièreté de l'arche avec la PTZ
  - Solutions à explorer :
    - Passer d'une séquence de 3 photos à 5 photos
    - Revoir l'enchaînement des positions de la PTZ
    - Essayer d'utiliser le script video à très basse vitesse

### Amélioration de la vitesse de traitement des images (surtout avec les vidéos)
- [x] **Optimisation de la détection YOLO avec Docker et TensorRT** :
  - Recherche et construction de l'image Docker : Création d'une image Docker spécialisée pour Jetson Orin (`Dockerfile.jetson`) basée sur `dustynv/l4t-pytorch:r36.2.0` pour bénéficier du support GPU natif. L'image intègre ROS 2 Humble, PyTorch avec CUDA, et toutes les dépendances nécessaires (Ultralytics, OpenCV, NumPy < 2.0 pour compatibilité avec cv_bridge). Cette approche permet d'isoler l'environnement de détection YOLO et d'utiliser efficacement le GPU du Jetson.
  - Optimisation par conversion TensorRT : Création du script `scripts/convert_to_tensorrt.sh` pour convertir le modèle PyTorch `best.pt` en format TensorRT `best.engine`, permettant une accélération significative de l'inférence sur Jetson (jusqu'à 3-5x plus rapide). Le script utilise le conteneur Docker pour effectuer la conversion avec les paramètres optimaux (half precision, device GPU, taille d'image 640x640).
  - Adaptation et création des scripts :
    - `scripts/image_subscriber_gpu.sh` : Script unifié qui délègue à `docker/launch_jetson.sh` pour lancer le nœud `image_subscriber` dans le conteneur Docker avec accès GPU
    - `docker/launch_jetson.sh` : Script de lancement du conteneur Docker avec configuration réseau host, montage des volumes, et vérification automatique des dépendances (ROS 2, Ultralytics, modèles YOLO)
    - `docker/Dockerfile.jetson` : Dockerfile optimisé pour Jetson avec installation de ROS 2 Humble et dépendances Python pour YOLO
  - Adaptation du code Python : Modification de `ros2_ws/src/image_transfer/image_transfer/image_subscriber.py` pour détecter et utiliser automatiquement le modèle TensorRT (`best.engine`) s'il est disponible, avec fallback sur `best.pt` si nécessaire.
- [x] **Désactivation des images de profondeur sur la ZED** :
  - Les nœuds de la ZED générant les nuages de points consomment beaucoup de mémoire GPU.
  - La désactivation permet de libérer des ressources pour l’analyse d’images avec YOLO.

### Arbre TF incohérent (TF statiques non implémentées => dérive obligatoire)
- [x] **Navigation et visualisation** :
  - Problèmes TF identifiés : Trajectoire du robot sur RViz n'est pas sur un plan horizontal
  - Caméra ZED2i : Cohérence des données pas encore vérifiée
  - **Solution** : Révision complète du filtre EKF (la VO `/zed/zed_node/odom` crée un conflit)
- [x] **Reconstruire l'arbre TF** :
  - Reprendre l'ensemble des frames et des transformations statiques pour garantir leur cohérence
  - Intégrer précisément les repères transmis par scout_base (pas utilisé jusqu'ici)
  - Mesurer et appliquer avec précision les distances réelles entre repères dans les TF statiques
  - Supprimer la frame `world` devenue redondante, car `odom` joue déjà ce rôle

### Scan LiDAR (impossible de créer une carte avec les scans fournis)
- [x] **Création d'une config custom** : `ros_launcher/configs/ydlidar_TG15.yaml` (config ingénieurs TechLab), pour améliorer la qualité des scans.
- **Amélioration du support LiDAR par impression 3D** : Nous avons imprimé en 3D une plateforme dédiée afin d'exploiter pleinement la capacité du LiDAR pour la création de cartes. À l'origine, le capteur était fixé entre quatre piliers, ce qui perturbait fortement les mesures et influençait de manière importante la qualité des scans. Désormais, le robot fonctionne avec deux modes : un mode avec la plateforme 3D pour la cartographie, et un mode sans la plateforme (LiDAR fixé entre les quatre piliers) pour l'utilisation en navigation, notamment pour la détection d'obstacles.
- [x] **Paramètre de montage du LiDAR** : Selon la position du LiDAR par rapport à l'avant de la tourelle, il faut paramétrer le LiDAR.
  - `laser_mount_yaw` — défaut π/2 pour le montage « normal »
  - `laser_mount_yaw:=0.0` pour création de carte (SLAM)

### Mise à jour config EKF + SLAM (amélioration de la création de carte)
- [x] **EKF** : Configuration du filtre de fusion dans `ekf_config.yaml` :
    - La pose et la vitesse du robot (`x`, `y`, `yaw`, `vx`, `vy`) sont obtenues uniquement depuis les roues via `/odom_robot` (aucune fusion avec la VO de la ZED ou d'autres capteurs).
    - L’IMU ZED n’est utilisée que pour la vitesse angulaire en lacet (`vyaw`) afin de limiter l'influence du bruit sur le yaw absolu.
    - Aucun `odom1` n’est présent, la ZED ne fusionne plus les vitesses.
    - Réglages complémentaires : fréquence du filtre à 15 Hz, history_length à 10, tuning précis des matrices de covariance et process_noise pour une trajectoire plus stable (voir détail dans `configs/ekf_config.yaml`).
- [x] **SLAM** : Paramétrage du nœud SLAM Toolbox dans `slam_config.yaml` pour assurer la compatibilité avec le LiDAR TG15 et le pipeline EKF :
    - Définition stricte des frames (`odom_frame: odom`, `base_frame: base_link`, `map_frame: map`).

## Seconde descente - Vendredi 10 avril 2026
- [x] **Réalisation** : Seconde descente dans les tunnels ANDRA effectuée avec succès
- [x] **Caméra 360** : Caméra 360 de nouveau prêtée par l'ANDRA, jeu de données supplémentaire créé
- [x] **Système opérationnel** :
  - Nouvelle séquence de prise de photos bien plus efficace (capture toute l'arche "rapidement")
  - Séquence de prise vidéo fonctionnelle
  - Traitement des images par GPU uniquement fonctionnel
  - Première tentative de cartographie des tunnels de l'ANDRA encourageante mais peu fiable (dérive du robot)
- [x] **Améliorations à réaliser sur la cartographie** :
  - Stabiliser la localisation du robot lors des scans
  - Trouver des paramètres LIDAR/SLAM/EKF qui permettent une cartographie de qualité

### Création de parcours avec évitement automatique
- [x] **Validation du mode AMCL + Nav2 pour la navigation sur carte** :
  - Lancement en mode `amcl` avec carte (`map_server` + `amcl`) et activation de Nav2 (`use_nav:=true`).
  - Validation des objectifs de navigation via RViz (`2D Nav Goal`) dans le repère `map`.
- [x] **Création d'un nœud mission pour exécuter un circuit** :
  - Remplacement du prototype `position_director.py` (contrôle local `/cmd_vel`) par une approche mission basée Nav2.
  - Nœud implémenté : `navigation_utils/trajectoire_mission.py`.
  - Fonction : lecture d'un fichier `traj.yaml` puis envoi séquentiel des goals `navigate_to_pose`.
- [x] **Mise en place des fichiers de trajectoire par carte** :
  - Dossier dédié : `ros2_ws/src/navigation_utils/trajectoire/`.
  - Fichiers créés depuis RViz en capturant `/goal_pose`, conversion quaternion -> yaw, puis structuration en YAML `waypoints`.
  - Boucle fermée supportée en répétant le point de départ à la fin de la liste.
- [x] **Ajout d'un arrêt d'urgence frontal en mission** :
  - Surveillance LiDAR `/scan` pendant l'exécution des goals.
  - Si obstacle frontal sous seuil (`emergency_stop_distance`), annulation du goal en cours et arrêt de la mission.

---

## Problèmes actuels / en cours de résolution

### Impossible de rendre le robot autonome (les piliers de la tourelle cachent trop le LiDAR)
- [x] **Conception d'une nouvelle tourelle** :
  - Les premiers tests de navigation AMCL révèlent que le LiDAR sera aussi essentiel lors de la navigation autonome. La localisation seule sur la carte ne suffit pas.
  - La tourelle actuelle ne permet pas une bonne utilisation du LiDAR et de la caméra PTZ simultanément.
  - Il faut créer une tourelle avec un plexiglas autour du LiDAR.
  - Problème : les scans ne passent à travers aucun matériau, même transparent.
  - Abandon de l'idée d'une nouvelle tourelle
- [ ] **Changer la position de la caméra** :
  - Nouvelle idée : nous allons positionner la caméra derrière la tourelle.
  - Elle doit seulement prendre des photos/vidéos d'une arche ; elle n'a pas besoin de voir devant elle.
  - Il faut finir la platine prototype du LiDAR et fabriquer un nouveau support pour la caméra.
  - Conception 3D de nouveaux support par Vincent
- [x] **Arret d'urgence** :
  - Les tests terrain ont montré trop de faux positifs LiDAR malgré plusieurs filtrages (angle, distance, confirmations temporelles/spatiales).
  - Décision actuelle : retrait de l'arrêt d'urgence logiciel dans `trajectoire_mission.py` pour éviter les arrêts intempestifs de mission.
  - Stratégie retenue : s'appuyer sur l'évitement Nav2 (comportement d'esquive jugé globalement fiable) plutôt que forcer un stop logiciel instable.
  - Action future si nécessaire : réintroduire un arrêt d'urgence uniquement avec une source capteur plus robuste / dédiée.

### Fusion de l'analyse et de la navigation
- [x] **Rendre les séquences autonomes** :
  - Objectif : suivre un circuit de waypoints tout en capturant photo/vidéo.
  - Nouveau package ROS2 `patrouille_autonome` créé dans `ros2_ws/src`.
  - Scripts d'orchestration ajoutés :
    - `fusion_photo_navigation` : lance `trajectoire_mission` + `sequence_photo`.
    - `fusion_video_navigation` : lance `trajectoire_mission` + `sequence_video`.
  - Paramétrage mission intégré : waypoint par défaut `ma_carte_traj.yaml`, `frame_id:=map`, option `--loop`.
  - Fin de mission : arrêt automatique de la séquence photo/vidéo, retour PTZ en preset Home.
  - Arrêt robuste (`Ctrl+C`) : shutdown des deux nœuds de fusion + commande de sécurité `cmd_vel=0`.
- [ ] **Retoucher les séquences**
  - Aligner `sequence_photo` et `sequence_video` sur un usage "capture only" quand elles sont lancées via `patrouille_autonome` (ne jamais concurrencer Nav2 sur `/cmd_vel`).
  - Revoir le timing PTZ/capture pour éviter les captures pendant les phases d'accélération/virage (fenêtre stable ou waypoint atteint).
  - Uniformiser le comportement de fin : arrêt propre des séquences, `cmd_vel=0`, retour PTZ Home, fermeture sans processus orphelin.
  - Clarifier les paramètres d'orchestration (distance/rythme de capture, loop, vitesse, waypoint file) et leurs valeurs par défaut.
- [ ] **Associer chaque média à la pose robot** :
  - Réviser les nœuds `show_pos.py`, `position_publisher.py`.
  - Comprendre comment fonctionne `report_fissures.py`.
  - Les intégrer au système.

### Pas de caméra 360° au TechLab (intégration impossible)
- [ ] **Disposer d'une caméra 360°** :
  - Modèle Ricoh Theta compatible avec ROS.
  - Le TechLab va peut-être acheter des Insta360.
  - Voir si un plugin open source existe.
- [ ] **Intégrer la caméra 360° au robot**

### Annotation du dataset
- [ ] **Quel logiciel utiliser ?**

---

## À faire

### Test et compréhension du projet
- [ ] Tester les nœuds de position sur une carte
- [ ] Tests de bout en bout du système complet

### Amélioration du modèle de détection
- [x] Dataset caméra 360 : Dataset créé
- [ ] Lucas et Adrien : Trier et annoter le dataset de la caméra 360
- [ ] Lucas et Adrien : Entraîner un modèle de détection sur image avec caméra 360
- [ ] Améliorer l'efficacité du robot avec le nouveau modèle
- [ ] Tester le nouveau modèle sur les images capturées

### Amélioration du positionnement
- [x] Améliorer l'estimation de position relative
- [ ] Utiliser les étiquettes au mur pour recalibrage ?

### Amélioration de la cartographie
- [x] Améliorer la cartographie des tunnels (actuellement fonctionne mal à cause des piliers qui entourent le LiDAR)
- [x] Tester la cartographie dans les tunnels réels
- [ ] Réduire les erreurs de divergence dues à la roue avant gauche
- [ ] Créer des cartes précises des galeries

---

## Objectif final

### Robot autonome complet
- [x] Robot autonome faisant des rondes dans les tunnels
- [ ] Système de mission/planification de parcours
- [ ] Système de localisation robuste combinant plusieurs méthodes

### Cartographie complète
- [ ] Cartographier tous les tunnels nécessaires
- [ ] Créer une carte globale des galeries
- [x] Intégrer la cartographie dans le système de navigation

### Analyse continue
- [ ] Analyse continue des images capturées par caméra 360
- [ ] Détection automatique des fissures pendant les rondes
- [ ] Génération de rapports automatiques

### Amélioration IA
- [ ] Étendre l'IA à tous les types de murs (actuellement restreinte au type GER)
- [ ] Entraîner le modèle pour chaque type de mur de la galerie
- [ ] Améliorer la segmentation des fissures

---

## Répartition des tâches

### Lucas et Adrien
- Entraînement du modèle de détection avec caméra 360
- Amélioration de l'efficacité de détection
- Création automatique de rapport sur l'état des fissures

### Vincent et Paul-Antoine
- Installation et configuration des drivers
- Documentation et rapports sur l'utilisation du robot
- Rendre le robot autonome dans les tunnels

---

## Notes techniques

### Problèmes connus (d'après README.md de l'année dernière)
- L'estimation de position relative est objectivement mauvaise (capteurs d'entrée de gamme, vibrations)
- La cartographie fonctionne mal à cause de la dérive du robot (roue avant gauche voilée)
- L'IA est restreinte à un unique type de mur (GER)

### Solutions suggérées (d'après README.md)
- Utiliser les étiquettes au sein de l'ANDRA pour se recalibrer
- Améliorer le positionnement (nombreux robots ont déjà résolu ce problème : Spot, Unitree GO2)
- Étendre l'entraînement de l'IA à tous les types de murs

---

**Dernière mise à jour** : 21 avril 2026
