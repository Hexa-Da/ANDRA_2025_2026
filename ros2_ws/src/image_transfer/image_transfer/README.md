# 🤖 Système de Détection de Fissures par Robot Mobile (ROS 2)

## Description générale

Ce projet met en œuvre un **système robotique autonome de détection de fissures ou de tags visuels**, basé sur **ROS 2**, combinant vision par ordinateur, intelligence artificielle, cartographie et contrôle matériel.

L’architecture est organisée autour de quatre blocs fonctionnels :

- 🧠 Vision et Intelligence Artificielle (le *cerveau*)
- 🗺️ Cartographie et reporting (la *mémoire*)
- 🦾 Contrôle matériel (le *corps*)
- 🧭 Positionnement et débogage

---

## 🧠 1. Vision et Intelligence Artificielle (Le Cerveau)

### image_publisher.py  
**Nœud ROS 2 :** `noeud_3`

**Rôle**  
Interface avec la caméra PTZ. Ce nœud récupère le flux vidéo, applique des corrections d’image et diffuse les images aux autres nœuds ROS.

**Fonctionnalités**
- Connexion au flux RTSP de la caméra PTZ (IP : `192.168.5.163`)
- Réglage de la luminosité, du contraste et du gamma via paramètres ROS
- Publication des images sur le topic `/photo_topic`
- Sauvegarde des images brutes dans `ros2_ws/images_capturees`

---

### image_subscriber.py  
**Nœud ROS 2 :** `noeud_5`

**Rôle**  
Nœud de détection utilisant une intelligence artificielle (YOLO) pour identifier des fissures ou des tags.

**Fonctionnalités**
- Abonnement aux topics :
  - `/photo_topic`
  - `/odometry/filtered`
- Utilisation du modèle YOLO `best.pt`
- Lorsqu’une détection est effectuée :
  - Publication de la position du robot sur `/position_detectee`
  - Message de type `geometry_msgs/Point`
- Dessin des bounding boxes sur l’image
- Sauvegarde des images annotées dans `ros2_ws/images_detectees`

---

## 🗺️ 2. Cartographie et Reporting (La Mémoire)

### report_fissures.py  
**Nœud ROS 2 :** `map_point_plotter`

**Rôle**  
Génération de rapports visuels indiquant la position des fissures détectées sur une carte du site.

**Fonctionnement**
- Écoute du topic `/position_detectee`
- Chargement de la carte statique définie dans `andra.yaml` et son image PGM
- Conversion des coordonnées du robot (mètres) en coordonnées pixels
- Ajout d’un point rouge sur la carte
- Sauvegarde de l’image finale `map_with_point_XXXX.png`

---

### test.py  
**Nœud ROS 2 :** `map_checker`

**Rôle**  
Outil de diagnostic pour la cartographie et la navigation.

**Fonctionnalités**
- Vérification de la validité du fichier YAML de la carte
- Vérification du bon fonctionnement du Map Server
- Contrôle de la publication du topic `/map`
- Aide au débogage lorsque la navigation ne démarre pas

---

## 🦾 3. Contrôle Matériel (Le Corps)

### ptz_controller.py  
**Nœud ROS 2 :** `ptz_controller`

**Rôle**  
Pilotage de la caméra PTZ Marshall CV-605.

**Fonctionnement**
- Communication via le protocole VISCA over IP
- Connexion TCP à l’adresse `192.168.5.163`
- Traduction des commandes ROS en commandes hexadécimales VISCA
- Topics utilisés :
  - `/ptz/cmd_vel` pour le mouvement
  - `/ptz/preset` pour les positions pré-enregistrées

---

## 🧭 4. Positionnement et Débogage

### show_pos.py  
**Nœud ROS 2 :** `odometry_subscriber`

**Rôle**  
Affichage en temps réel de la position du robot.

**Fonctionnement**
- Lecture du topic `/odometry/filtered`
- Affichage des coordonnées X, Y et Z dans le terminal toutes les secondes

---

### position_publisher.py  
**Nœud ROS 2 :** `position_subscriber`

⚠️ **Attention au nom**  
Le fichier s’appelle `position_publisher.py` mais la classe définie est `PositionSubscriber`.

**Rôle**  
Nœud de journalisation synchronisée entre détection et position.

**Fonctionnement**
- Écoute :
  - `detection_status` (booléen)
  - `/odometry/filtered`
- Si `detection_status` est vrai, la position du robot est affichée dans les logs

---

## 🧩 Architecture globale

Caméra PTZ  
→ image_publisher  
→ `/photo_topic`  
→ image_subscriber (YOLO)  
→ `/position_detectee`  
→ report_fissures  
→ Carte annotée

---

## ✅ Points forts

- Architecture ROS 2 modulaire et claire
- Détection IA synchronisée avec la position du robot
- Génération automatique de rapports cartographiques
- Contrôle matériel bas niveau de la caméra PTZ
- Outils dédiés au débogage et au diagnostic
