# TODO - Projet ANDRA 2025-2026

## Contexte du projet

Groupe de 4 étudiants en projet industriel avec l'ANDRA. Mission : rendre le robot Agilex Scout Mini autonome dans les galeries de l'ANDRA pour effectuer une analyse des fissures sur les murs via une caméra 3D.

**Transmission d'année en année** : Ce projet est transmis d'année en année parmi les étudiants. Le but est de s'approprier les avancées réalisées l'année dernière pour reproduire leurs résultats finaux, puis réalisé les mission demandé par l'ANDRA.

## Échéances importantes

- **Soutenance mi-parcouts** : 23 janvier 2026
- **Première descente dans les tunnels** : début février 2026
- **Autres descente** : ?
- **Soutenance fianle** : 12 juin 2026

---

## Ce qui a été fait

### Accès et ressources
- [x] Accès au dépôt GitHub 
- [x] Accès au drive hébergeant le modèle YOLO11 
- [x] Accès au robot Agilex Scout Mini qui est au TechLab 

### Infrastructure et scripts
- [x] Analyse et compréhension de l'indrastructure et des scripts
- [x] Etablir un worflow simple pour toute l'équipe
- [x] Création d'un script d'initialisation (`scripts/setup.sh`)
- [x] Création d'un script de compilation (`scripts/build.sh`)
- [x] Création d'un script de lancement (`scripts/launch.sh`)
- [x] Organisation de la structure du projet (workspaces, dépendances, configurations)

### Documentation
- [x] Documentation complète du démarrage (`DEMARRAGE_ROBOT.md`)
- [x] Guide d'utilisation des scripts
- [x] Documentation des nœuds ROS2

### Nœuds ROS2 fonctionnels (les neouds tournent mais les données renvoyées n'ont pas été verifiées)
- [x] `image_publisher` : Capture des images depuis la caméra PTZ
- [x] `image_subscriber` : Détection YOLO des fissures, sauvegarde des images détectées
- [x] `position_publisher` : Affichage de la position du robot
- [x] `report_fissures` : Traçage des positions détectées sur la carte
- [x] `ptz_controller` : Contrôle PTZ de la caméra Marshall CV-605 via protocole VISCA over IP 
- [x] Correction des erreurs de shutdown dans les nœuds Python (gestion d'exception dans le finally)

### Configuration navigation
- [x] Configuration SLAM Toolbox (`ros_launcher/slam_config.yaml`)
- [x] Configuration AMCL (`ros_launcher/amcl_config.yaml`)
- [x] Configuration EKF pour fusion des capteurs (`ros_launcher/ekf_config.yaml`)
- [x] Launch file principal (`ros_launcher/navigation_stack.launch.py`)
- [x] Transforms TF statiques (base_link → zed_camera_link, base_link → laser_frame)

### Configuration LIDAR
- [x] Connexion au port série `/dev/ttyTHS1` fonctionnelle
- [x] Configuration avec fichiers de modèle (G2.yaml, G4.yaml testés)
- [x] Intégration dans le launch file avec paramètres configurables

---

## En cours / Problèmes rencontrés

### Réinstallation complète du robot
- [ ] **Problème** : L'image qui avait été créée sur le robot l'année dernière a été supprimée sans sauvegarde
- [ ] **Action** : Refaire toute la réinstallation et sourçage des drivers ainsi que leur configuration

### Drivers manquants
- [x] **scout_base** : Package non trouvé (nécessaire pour l'odométrie des roues)
  - Repository probable : https://github.com/agilexrobotics/scout_ros2
  - Topic attendu : `/odom_robot`

### Problème LIDAR
- [x] Connexion au port série réussie (`/dev/ttyTHS1`)
- [ ] **Erreur** : `Error, cannot retrieve Lidar health code -1`
- [ ] **Erreur** : `Fail to get baseplate device information!`
- [ ] **Erreur** : `Failed to start scan mode -1`
- [ ] **Statut** : Connexion OK mais scan ne démarre pas
- [ ] **Tests effectués** : 
  - Modèles G2.yaml et G4.yaml testés
  - Baudrates 115200 et 230400 testés
  - Port série `/dev/ttyTHS1` confirmé 
- [ ] **Action** : Vérifier connexion matérielle (alimentation, câble) ou décider de continuer sans LIDAR

### Problème Zed 2
- [x] **zed_wrapper** : Package installé et fonctionnel
  - ZED SDK installé dans /usr/local/zed
  - zed_msgs installé via apt
  - zed_wrapper compilé dans dependencies/zed-ros2-wrapper
  - Caméra ZED 2i détectée (S/N 32802052)
  - Topics publiés : /zed/zed_node/odom, /zed/zed_node/imu/data, /zed/zed_node/rgb/color/rect/image
  - Configuration EKF mise à jour pour utiliser les données ZED

### Problèmes au lancement
- [x] **LIDAR** : Connexion OK mais scan ne démarre toujours pas (possiblement une erreurs matérielles)
- [x] **scout_base** : ✅ **RÉSOLU**
  - **Problème initial** : le noeud s'arrete immédiatement lors du lancement
  - **Configuration** : Ajout du paramètre `odom_topic_name:=odom_robot` pour correspondre à la configuration EKF
  - **Problème identifié** : Le driver `ugv_sdk` ne reconnaissait pas "agilex" comme un port CAN valide (vérification stricte du nom contenant "can")
  - **Solution** : Modification du code source `scout_base_ros.cpp` pour accepter "agilex" en plus des noms contenant "can"
  - **État actuel** : ✅ Le nœud démarre correctement et communique avec le robot via l'interface CAN `agilex`

### Problème Caméra PTZ
- [x] ✅ **RÉSOLU** : Caméra PTZ maintenant accessible et fonctionnelle
  - **Problème initial** : Caméra PTZ inaccessible sur `192.168.5.163`
  - **Cause** : La Jetson n'était pas sur le même sous-réseau que la caméra PTZ (192.168.5.x)
  - **Solution** : Configuration de l'adresse IP statique sur l'interface Ethernet `enP8p1s0`
  - **Configuration réseau** : Interface configurée avec `192.168.5.100/24` pour accéder à la caméra `192.168.5.163`
  - **Résolution des conflits** : Suppression des routes conflictuelles avec le WiFi
  - **État actuel** : ✅ La caméra PTZ est accessible, les images sont capturées et publiées sur `/photo_topic`
- [x] ✅ **RÉSOLU** : Conflit d'adresse IP entre Ethernet et WiFi après redémarrage
  - **Problème** : Après redémarrage du robot, la configuration réseau n'était plus active
  - **Cause** : Le WiFi (`wlP1p1s0`) obtenait aussi l'adresse `192.168.5.100`, créant un conflit de routes
  - **Symptôme** : Les paquets vers `192.168.5.163` partaient via WiFi au lieu d'Ethernet
  - **Solution** : Création du script `scripts/ptz-network-setup.sh` qui :
    - Configure automatiquement l'IP sur `enP8p1s0`
    - Supprime les routes WiFi conflictuelles vers `192.168.5.0/24`
    - Vérifie que la route passe bien par Ethernet
  - **État actuel** : ✅ Script créé, peut être intégré dans un service systemd pour configuration automatique
  
- [x] ✅ **RÉSOLU** : Contrôle PTZ fonctionnel via VISCA over IP
  - **Caméra** : Marshall CV-605 (5x HD60 IP PTZ Camera with 3GSDI)
  - **Protocole** : VISCA over IP sur le port 1259 (port par défaut selon documentation)
  - **Nœud créé** : `ptz_controller` dans le package `image_transfer`
  - **Topics** :
    - `/ptz/cmd_vel` (geometry_msgs/Twist) : Contrôle pan/tilt continu
    - `/ptz/preset` (std_msgs/Int32) : Appel de presets (0-127) ou commandes spéciales (-1: Home, -2: Reset)
  - **Format VISCA** : Implémentation selon documentation Marshall CV-605
    - Header : `0x80 + camera_address` (adresse 1 par défaut)
    - Pan-Tilt Drive : `0x01 0x06 0x01 VV WW DD DD` où VV=pan speed (1-18), WW=tilt speed (1-14), DD DD=direction
  - **Paramètres configurables** : `ptz_ip`, `visca_port`, `camera_address`
  - **État actuel** : ✅ Contrôle PTZ fonctionnel, caméra répond aux commandes de mouvement

### Améliorations du système de lancement
- [x] **Options de configuration** : Ajout d'options pour désactiver des composants
  - `enable_lidar:=false` : Désactiver le LIDAR
  - `enable_scout:=false` : Désactiver Scout Base
  - `enable_zed:=false` : Désactiver la caméra ZED
  - `enable_ptz:=false` : Désactiver la caméra PTZ
  - Permet de tester le système même si un composant pose problème
  
---

## À faire - Court terme (avant première descente debut février)

### Configuration et installation
- [ ] Finaliser la réinstallation de l'image du robot
- [x] Installer/configurer le driver `scout_base` ✅ (Installé, configuré et fonctionnel)
- [x] Installer/configurer le driver `zed_wrapper` ✅ (Installé et fonctionnel)
- [x] Configurer la caméra PTZ ✅ (Réseau configuré, nœuds fonctionnels)
- [ ] Résoudre le problème LIDAR ou documenter la décision de continuer sans

### Reproduire les résultats de l'année dernière
- [ ] Robot capable d'avancer en ligne droite pendant 1 mètre
- [ ] Robot capable de s'arrêter pour prendre une image
- [ ] Robot capable de recommencer le cycle
- [ ] Robot capable de prendre une carte en entrée et d'estimer sa position (AMCL)

### Préparation première descente
- [ ] Tester le système complet dans l'environnement TechLab
- [ ] Prendre des photos avec la caméra 360 dans les tunnels
- [ ] Documenter les résultats de la première descente

---

## À faire - Moyen terme (avant les autres descente)

### Amélioration du modèle de détection
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
- [ ] Navigation autonome avec ligne jaune au sol (détection via ZED2)
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

### Configuration CAN
- [x] Interface CAN identifiée : `agilex` (configuration TechLab)
- [x] Service systemd configuré : `agilex-handler.service`
- Commande manuelle : `sudo ip link set agilex up type can bitrate 500000`
- [x] Launch file mis à jour pour utiliser `port_name:=agilex`

### Modèle YOLO
- Version utilisée : YOLOv11
- Fichier : `ros2_ws/models/best.pt`
- Avantage : Pas besoin de s'embêter avec PyTorch ou CUDA, tout est géré par défaut

### Configuration Caméra PTZ
- [x] **Réseau** : Configuration de l'adresse IP statique sur `enP8p1s0` (192.168.5.100/24)
- [x] **Accès RTSP** : Caméra accessible sur `rtsp://admin:admin@192.168.5.163:554/live/av0`
- [x] **Nœud image_publisher** : Paramètres configurables (IP, port, ajustements d'image, sauvegarde)
- [x] **Ajustement d'image** : Luminosité, contraste, gamma configurables via paramètres ROS2
- [x] **Mode automatique d'ajustement** : Implémentation du mode auto avec CLAHE pour ajustement automatique 
- [x] **Sauvegarde** : Deux dossiers distincts (toutes les images + images avec détection)
- [x] **Contrôle PTZ** : Nœud `ptz_controller` fonctionnel via VISCA over IP
- [x] **Script de configuration réseau automatique** : `scripts/ptz-network-setup.sh` (Conflit adresse IP) 
- [x] **Script d'enregistrement vidéo** : `video_launcher/script.sh`

---

**Dernière mise à jour** : 12 Janvier 2026
