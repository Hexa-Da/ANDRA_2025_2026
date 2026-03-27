# TODO - Projet ANDRA 2025-2026

## Contexte du projet

Projet réalisé par un groupe de 4 étudiants en partenariat avec l'ANDRA.

Mission : rendre le robot Agilex Scout Mini autonome dans les galeries de l'ANDRA pour effectuer une analyse des fissures sur les murs via une caméra 3D.

**Transmission d'année en année** : Ce projet est transmis d'année en année parmi les étudiants. Le but est de s'approprier les avancées réalisées l'année dernière pour reproduire leurs résultats finaux, puis réalisé les missions supplémentaires demandé par l'ANDRA.

## Échéances importantes

- **Soutenance mi-parcouts** : 23 janvier 2026
- **Première descente dans les tunnels** : 6 février 2026
- **Autres descente** : ?
- **Soutenance fianle** : 12 juin 2026

---

## Ce qui a été fait

### Accès et ressources
- [x] Accès au dépôt GitHub 
- [x] Accès au drive hébergeant le modèle YOLO11 
- [x] Accès au robot Agilex Scout Mini du TechLab 

### Infrastructure et scripts
- [x] Analyse et compréhension de l'infrastructure et des scripts
- [x] Etablir un worflow simple pour toute l'équipe
- [x] Création d'un script d'initialisation (`scripts/setup.sh`)
- [x] Création d'un script de compilation (`scripts/build.sh`)
- [x] Création d'un script de lancement (`scripts/launch.sh`)
- [x] Création d'un script de mise en réseau de la PTZ (`scripts/ptz-network-setup.sh`)
- [x] Réecriture du script video, avec sauvegarde dans `video_output/` (`video/script.sh`)
- [x] Organisation de la structure du projet (workspaces, dépendances, configurations/launcher)
- [x] Nettoyage du code, fichier non utilisés ou dépréciés

### Documentation
- [x] Documentation complète du démarrage (`DEMARRAGE_ROBOT.md`)
- [x] Guide connexion réseau Techlab vs Hotspot (`docs/HOTSPOT.md`)
- [x] Explication de la structure du projet (`STRUCTURE.md`)
- [x] Guide d'utilisation des scripts (`SCRIPTS.md`)
- [x] Guide de debogage (`DEBUG.md`)
- [x] Guide de contrôle PTZ (`PTZ_PRESETS.md`)
- [x] Guide de visualisation RViz2 (`VISUALISATION.md`)
- [x] Documentation des nœuds ROS2
- [x] Documentation détection YOLO et TensorRT (`docs/DETECTION_YOLO.md`)

### Verification/Implémentation des nœuds ROS2  
- [x] `image_publisher` : Capture des images depuis la caméra PTZ, sauvegarde dans `images_capturees/`
- [x] `video_publisher` : Traitement des vidéos enregistrées, extraction d'images et publication sur `/photo_topic`
- [x] `image_subscriber` : Détection YOLO des fissures, sauvegarde dans `images_detectees/`
- [x] `position_publisher` : Affichage de la position du robot
- [x] `report_fissures` : Traçage des positions détectées sur la carte
- [x] `ptz_controller` : Contrôle PTZ de la caméra Marshall CV-605 via protocole VISCA over IP 
- [x] `sequence_photo` : Automatisation de la séquence de mouvement et captures PTZ en boucle (5 captures)
- [x] `sequence_video` : Enregistrement vidéo continu avec balayage PTZ automatique

### Configuration navigation
- [x] Configuration SLAM Toolbox (`ros_launcher/slam_config.yaml`)
- [x] Configuration AMCL (`ros_launcher/amcl_config.yaml`)
- [x] Configuration EKF pour fusion des capteurs (`ros_launcher/ekf_config.yaml`)
- [x] Launch file principal (`ros_launcher/navigation_stack.launch.py`)
- [x] Transforms TF statiques (base_link → zed_camera_link, base_link → laser_frame)
- [x] Visualisation trajet en RViz (Fixed Frame `odom`, Path `/robot_path`, nœud `odom_to_path`)

---

## En cours / Problèmes rencontrés

### Réinstallation complète du robot
- [ ] **Problème** : L'image qui avait été créée sur le robot l'année dernière a été supprimée sans sauvegarde
- [x] **Action** : Refaire toute la réinstallation et sourçage des drivers ainsi que leur configuration 
- [ ] **Objectif** : Atteindre les résultats finaux de l'année dernière

### Problème LIDAR
- [x] Connexion au port série (`/dev/ttyTHS1`)
- [x] **Diagnostic communication LiDAR** : 
  - Scan de baudrates effectués : 115200, 128000 (X4), 230400 (G4) sans réponse
  - Tentative activation forcée via DTR/RTS sans succès
  - LED s'allume brièvement puis s'éteint (mise en sécurité ou coupure alimentation)
- [x] **Diagnostic du chipset (Carte Radar_Con)** : 
  - Le port Micro USB est arraché, ce qui empêche l’alimentation du moteur et donc le lancement du scan
  - Test de loopback concluant (TX/RX opérationnels), le reste des composants fonctionne donc correctement
  - Un branchement de secours via le port USB-C a été tenté et s’est avéré fonctionnel
- [x] **Problème résolu après intervention sur le chipset** : ✅ **RÉSOLU**
  - **Modification de l’alimentation** : Passage au port USB-C `/dev/ttyUSB0` pour fournir suffisamment de puissance au LIDAR.
  - **Correction de la configuration** : Le mauvais fichier de configuration (G4.yaml) était utilisé alors que le LIDAR est en réalité un modèle TG15.
  - **Actions menées** :
    - Remplacement du port `/dev/ttyTHS1` par `/dev/ttyUSB0`
    - Suppression de l’ancien fichier de configuration local (`ros_launcher/ydlidar_config.yaml`)
    - Adoption du fichier officiel `TG.yaml` du package `ydlidar_ros2_driver`
    - Simplification du launch file afin d’utiliser directement la configuration officielle fournie par le package
  - **Résultat** : Le LIDAR démarre normalement et publie sur `/scan` avec les paramètres attendus.
- [x] **État actuel** : ✅ LIDAR en service
  - Modèle : TG15 (Model Code 100)
  - Port utilisé : `/dev/ttyUSB0` (détection automatique)
  - Baudrate : 512000
  - Fréquence de balayage : 10 Hz
  - Taux d’échantillonnage : 20 kHz
  - Publication sur `/scan` : ✅ Ok

### Problème Zed 2
- [x] **zed_wrapper** : Package installé et fonctionnel
  - ZED SDK installé dans /usr/local/zed
  - zed_msgs installé via apt
  - zed_wrapper compilé dans dependencies/zed-ros2-wrapper
  - Caméra ZED 2i détectée (S/N 32802052)
  - Topics publiés : /zed/zed_node/odom, /zed/zed_node/imu/data, /zed/zed_node/rgb/color/rect/image
  - Configuration EKF mise à jour pour utiliser les données ZED
- [ ] **Vérification cohérence données** : Cohérence des données renvoyées par la caméra ZED2i pas encore vérifiée

### Problèmes scout_base
- [x] **Package** : Package non trouvé (nécessaire pour l'odométrie des roues)
  - Repository probable : https://github.com/agilexrobotics/scout_ros2
  - Topic attendu : `/odom_robot`
- [x] **Arret du neoud** :
  - **Configuration** : Ajout du paramètre `odom_topic_name:=odom_robot` pour correspondre à la configuration EKF
  - **Problème identifié** : Le driver `ugv_sdk` ne reconnaissait pas "agilex" comme un port CAN valide (vérification stricte du nom contenant "can")
  - **Solution** : Modification du code source `scout_base_ros.cpp` pour accepter "agilex" en plus des noms contenant "can"
  - **État actuel** : ✅ Le nœud démarre correctement et communique avec le robot via l'interface CAN `agilex`

### Problème Caméra PTZ
- [x] **Adresse statique introuvable** : Caméra PTZ inaccessible sur `192.168.5.163`
  - **Cause** : La connection ethernet de la Jetson n'était pas sur le même sous-réseau que la caméra PTZ (192.168.5.x)
  - **Solution** : Configuration de l'adresse IP statique sur l'interface Ethernet `enP8p1s0`
  - **Configuration réseau** : Interface configurée avec `192.168.5.100/24` pour accéder à la caméra `192.168.5.163`
- [x] **Conflits** : Le WiFi (`wlP1p1s0`) obtenait aussi l'adresse `192.168.5.100`, créant un conflit de routes
  - **Solution** : Création du script `scripts/ptz-network-setup.sh` qui :
    - Configure automatiquement l'IP sur `enP8p1s0`
    - Supprime les routes WiFi conflictuelles vers `192.168.5.0/24`
    - Vérifie que la route passe bien par Ethernet
  - **État actuel** : ✅ La caméra PTZ est accessible, les images sont capturées et publiées sur `/photo_topic`
- [x] **Controle PTZ** : Contrôle PTZ fonctionnel via VISCA over IP port 1259
  - **Caméra** : Marshall CV-605 (5x HD60 IP PTZ Camera with 3GSDI)
  - **Nœud créé** : `ptz_controller` dans le package `image_transfer`
  - **Topics** :
    - `/ptz/cmd_vel` (geometry_msgs/Twist) : Contrôle pan/tilt continu (utilisé par `sequence_photo` et `sequence_video`)
    - `/ptz/preset` (std_msgs/Int32) : Preset Home (-1) pour retour au centre et Reset (0) pour le recallibrage
  - **Format VISCA** : Implémentation selon documentation Marshall CV-605
    - Header : `0x80 + camera_address` (adresse 1 par défaut)
    - Pan-Tilt Drive : `0x01 0x06 0x01 VV WW DD DD` où VV=pan speed (1-18), WW=tilt speed (1-14), DD DD=direction
    - Home : `0x01 0x06 0x04` pour retour au centre
  - **État actuel** : ✅ Contrôle PTZ fonctionnel, caméra répond aux commandes de mouvement et preset
- [x] **Automatisation séquence robot** : Création des nœuds `sequence_photo` et `sequence_video`
  - **sequence_photo** : Avance 1m, arrêt 30s, 5 captures PTZ (Gauche, Haut×3, Droite, Haut), boucle
  - **sequence_video** : Vidéo continue + balayage PTZ horizontal sinusoïdal + robot à 0.06 m/s
  - **Contrôle PTZ** : `/ptz/cmd_vel` pour les mouvements ; preset Home (-1) ou socket VISCA direct à l'arrêt (Ctrl+C)
  - **Paramètres configurables** : Durées de mouvement, stabilisation, retour au centre
  - **État actuel** : ✅ Séquence automatique fonctionnelle, arrêt propre avec STOP puis HOME via socket

### Améliorations du système de lancement
- [x] **Options de configuration** : Ajout d'options pour désactiver des composants
  - `enable_lidar:=false` : Désactiver le LIDAR
  - `enable_scout:=false` : Désactiver Scout Base
  - `enable_zed:=false` : Désactiver la caméra ZED
  - `enable_ptz:=false` : Désactiver la caméra PTZ
  - Permet de tester le système même si un composant pose problème

### Configuration visualisation RViz2 avec Docker
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
    - **Fonctionnalités** :
      - Abonnement à `/odometry/filtered` (odométrie filtrée par EKF)
      - Publication du Path sur `/robot_path` pour visualiser le trajet du robot
      - Publication TF `odom → base_link` pour compléter l'arbre de transformations
    - **Paramètres** : Ajout d'un point au Path tous les 10 cm minimum (`min_distance = 0.1`)
  - **Configuration RViz2** : Mise à jour de `ros_launcher/config.rviz`
    - **Fixed Frame** : Changé de `map` à `odom` pour permettre la visualisation même si la carte n'est pas encore créée (mode SLAM initial)
    - **Display RobotPath** : Ajout d'un display Path pour visualiser `/robot_path` (couleur orange, style Lines)
    - **Displays existants** : Conservation de tous les displays précédents (Grid, TF, Map, LaserScan, PoseArray, Polygon, Pose)
  - **Utilité** : Permet de visualiser le trajet du robot même en mode SLAM avant que la carte ne soit créée
  - **Frame world** : Ajout de la frame `world` dans l'arbre TF
  - **Reconstruction de l'arbre TF** : Réorganisation de l'arbre des transformations pour intégrer `world` et clarifier les liens entre frames
  - **Conflit avec le nœud odom_to_path** : Le nœud `odom_to_path` publiais aussi `odom → base_link` ; à prendre en compte dans la reconstruction TF (éviter doublon ou conflit avec l'EKF)
  - **Objectif** : Constater la dérive de trajectoire entre les différents capteurs (odométrie roues, ZED, EKF fusionné) pour diagnostiquer le positionnement
- [ ] **Navigation et visualisation** (etat de la prise en main après première descente) :
  - Familiarisation avec RViz2 en cours
  - **Problèmes TF identifiés** : Trajectoire du robot sur RViz n'est pas sur un plan horizontal
  - **Caméra ZED2i** : Cohérence des données pas encore vérifiée

### Config Hotspot pour connexion dans les tunnels ANDRA
- [x] **Contexte** : Dans les tunnels, pas de Techlab-wifi ; le robot doit être joignable via son hotspot (réseau créé par la Jetson).
- [x] **À valider** : Vérifier que le hotspot `JetsonWIFI` (mdp `depinfonancy`) se lance bien quand le robot ne capte pas le Techlab ; connexion SSH via `ssh techlab@orin2.local` (ne pas utiliser `192.168.40.101` en mode hotspot).
- [x] **À tester** : Connexion et stabilité SSH depuis un PC portable connecté au hotspot du robot, dans un environnement extérieur au labo.
- [x] **Référence** : Procédure détaillée dans `docs/HOTSPOT.md` (mode Terrain, commandes `nmcli`).

### Inventaire et analyse hardware
- [x] **Inventaire complet** : Paul-Antoine a effectué un inventaire de l'ensemble des composants matériels du robot et a organisé le câblage interne de la tourelle.
- [x] **Étude de la structure de la tourelle** : Démontage puis remontage de la tourelle pour :
  - Identifier les composants effectivement utilisé ou non
  - Comprendre le rôle et l'utilité de chaque élément
  - Effectuer des tests individuels sur chaque capteur

### Première descente - Vendredi 6 février 2026
- [x] **Réalisation** : Première descente dans les tunnels ANDRA effectuée avec succès
- [x] **Caméra 360** : Caméra 360 prêtée par l'ANDRA, dataset créé (à trier et annoter)
- [x] **Système opérationnel** : 
  - ✅ Séquence de prise de photo fonctionnelle comme prévu
  - ✅ Système de Hotspot opérationnel
  - ✅ Tous les nœuds ROS2 fonctionnels
- [x] **Incident matériel** : Alimentation de la Jetson arrachée suite à une mauvaise manipulation
  - **Réparation** : Paul-Antoine a réparé la carte en changeant l'alimentation le week-end suivant
  - **État** : Robot de nouveau opérationnel rapidement
- [ ] **Constats sur la séquence PTZ** :
  - Séquence complète dans une galerie assez longue
  - Séquence actuelle (3 photos) ne couvre pas l'entièreté de l'arche avec la PTZ
  - **Solutions à explorer** :
    - Passer d'une séquence de 3 photos à 5 photos
    - Revoir l'enchaînement des positions de la PTZ
    - Essayer d'utiliser le script video à très basse vitesse

- [x] **Optimisation de la détection YOLO avec Docker et TensorRT** :
  - **Recherche et construction de l'image Docker** : Création d'une image Docker spécialisée pour Jetson Orin (`Dockerfile.jetson`) basée sur `dustynv/l4t-pytorch:r36.2.0` pour bénéficier du support GPU natif. L'image intègre ROS 2 Humble, PyTorch avec CUDA, et toutes les dépendances nécessaires (Ultralytics, OpenCV, NumPy < 2.0 pour compatibilité avec cv_bridge). Cette approche permet d'isoler l'environnement de détection YOLO et d'utiliser efficacement le GPU du Jetson.
  - **Optimisation par conversion TensorRT** : Création du script `scripts/convert_to_tensorrt.sh` pour convertir le modèle PyTorch `best.pt` en format TensorRT `best.engine`, permettant une accélération significative de l'inférence sur Jetson (jusqu'à 3-5x plus rapide). Le script utilise le conteneur Docker pour effectuer la conversion avec les paramètres optimaux (half precision, device GPU, taille d'image 640x640).
  - **Adaptation et création des scripts** : 
    - `scripts/image_subscriber_gpu.sh` : Script unifié qui délègue à `docker/launch_jetson.sh` pour lancer le nœud `image_subscriber` dans le conteneur Docker avec accès GPU
    - `docker/launch_jetson.sh` : Script de lancement du conteneur Docker avec configuration réseau host, montage des volumes, et vérification automatique des dépendances (ROS 2, Ultralytics, modèles YOLO)
    - `docker/Dockerfile.jetson` : Dockerfile optimisé pour Jetson avec installation de ROS 2 Humble et dépendances Python pour YOLO
  - **Adaptation du code Python** : Modification de `ros2_ws/src/image_transfer/image_transfer/image_subscriber.py` pour détecter et utiliser automatiquement le modèle TensorRT (`best.engine`) s'il est disponible, avec fallback sur `best.pt` si nécessaire. 

---

## À faire - Court terme (avant première descente debut février)

### Configuration et installation
- [x] Finaliser la réinstallation de l'image du robot
- [x] Installer/configurer le driver `scout_base` (Installé, configuré et fonctionnel)
- [x] Installer/configurer le driver `zed_wrapper` (Installé et fonctionnel)
- [x] Configurer la caméra PTZ (Réseau configuré, nœuds fonctionnels)
- [x] Résoudre le problème LIDAR (Résolu le 18 février 2026)

### Test et compréhension du projet
- [x] Tester la configuration de la navigation
- [ ] Crée une map de test
- [ ] Tester les noeuds de position du cette map
- [ ] Tests de bout en bout du système complet
- [ ] Validation des transformations TF (vérifier toutes les transformations)
- [x] Tests de performance des nœuds (fréquence de publication, latence)

### Reproduire les résultats de l'année dernière
- [x] Robot capable d'avancer en ligne droite pendant 1 mètre
- [x] Robot capable de s'arrêter pour prendre une image
- [x] Robot capable de recommencer le cycle (séquence automatique avec `sequence_photo` ou `sequence_video`)
  - Séquence PTZ : gauche → haut → droite avec captures
  - Retour au centre via preset Home (-1)
  - Boucle automatique de la séquence complète
- [ ] Robot capable de prendre une carte en entrée et d'estimer sa position (AMCL)

### Préparation première descente
- [x] Tester le système complet dans l'environnement TechLab
- [x] Prendre des photos avec la caméra 360 dans les tunnels
- [x] Documenter les résultats de la première descente (cf section "Première descente - Vendredi 6 février 2026")

---

## À faire - Moyen terme (avant les autres descente)

### Amélioration du modèle de détection
- [x] **Dataset caméra 360** : Dataset créé lors de la première descente (6 février 2026) - à trier et annoter
- [ ] **Lucas et Adrien** : Trier et annoter le dataset de la caméra 360
- [ ] **Lucas et Adrien** : Entraîner un modèle de détection sur image avec caméra 360
- [ ] Améliorer l'efficacité du robot avec le nouveau modèle
- [ ] Tester le nouveau modèle sur les images capturées

### Amélioration du positionnement (propositions Éliott)
- [ ] Utiliser les étiquettes au mur pour recalibration
- [ ] Améliorer l'estimation de position relative
- [ ] Réduire les erreurs de divergence dues aux vibrations

### Amélioration de la cartographie
- [ ] Améliorer la cartographie des tunnels (actuellement fonctionne mal à cause du mauvais positionnement)
- [ ] Créer des cartes précises des galeries
- [ ] Tester la cartographie dans les tunnels réels

### Navigation autonome de base
- [ ] Développer la navigation autonome au-delà de "avancer en ligne droite"
- [ ] Implémenter la détection et évitement d'obstacles si nous continuons avec le LIDAR

---

## À faire - Long terme (objectif final)

### Robot autonome complet
- [ ] Robot autonome faisant des rondes dans les tunnels
- [ ] Système de mission/planification de parcours

### Cartographie complète
- [ ] Cartographier tous les tunnels nécessaires
- [ ] Créer une carte globale des galeries
- [ ] Intégrer la cartographie dans le système de navigation

### Navigation avancée
- [ ] Navigation autonome avec ligne jaune au sol (détection via ZED2i)
- [ ] Navigation avec étiquettes au mur pour repérage dans les tunnels
- [ ] Système de localisation robuste combinant plusieurs méthodes

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
- [ ] Entraînement du modèle de détection avec caméra 360
- [ ] Amélioration de l'efficacité de détection

### Vincent et Paul-Antoine
- [ ] Configuration et tests du robot
- [ ] Installation et configuration des drivers
- [ ] Documentation et rapports sur l'utilisation du robot

---

## Notes techniques

### Problèmes connus (d'après README.md de l'année dernière)
- L'estimation de position relative est objectivement mauvaise (capteurs d'entrée de gamme, vibrations)
- La cartographie fonctionne mal à cause du mauvais positionnement
- L'IA est restreinte à un unique type de mur (GER)

### Solutions suggérées (d'après README.md)
- Utiliser les étiquettes au sein de l'ANDRA pour se recalibrer
- Améliorer le positionnement (nombreux robots ont déjà résolu ce problème : Spot, Unitree GO2)
- Étendre l'entraînement de l'IA à tous les types de murs

---

**Dernière mise à jour** : 19 Février 2026
