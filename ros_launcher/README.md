# Contexte
Ce travail a été réalisé par les étudiants en projet industrie en partenariat avec l'ANDRA sur l'année scolaire 2024-2025. Il consiste en la mise en place du déplacement et de la détection de fissures de façon autonome dans les galeries du site de Bure.

# Equipement 
Ce projet se base sur un robot Agilex Scout Mini équipé d'une ZED2 (caméra de profondeur), d'un LIDAR, d'une caméra 360° (dite PTZ) et d'une Jetson Orin Nano.
L'objectif est ici de combiner l'ensemble des capteurs d'estimation de position (vélocité des roues, IMU intégré de la ZED2, LIDAR) pour créer une carte de l'environnement et d'estimer la position du robot. A partir de la, une IA reconnaît la fissure via le flux vidéo qui provient de la PTZ.

# Ce qui a été fait 
- L'IA est convaincante et fonctionne plutôt bien. 
- Le robot est capable de prendre une carte en entrée et d'estimer sa position 
- Le robot est capable d'avancer en ligne droite pendant 1 mètre, s'arrêter pour prendre une image puis recommencer

# Ce qui marche pas / mal
- L'estimation de position relative est objectivement mauvaise, cela est principalement dû au fait que les capteurs sont d'entrées de gamme et ne sont pas fait pour se reposer uniquement sur eux (ils sont censés être utilisés en tant que complément par exemple avec un GPS). Nous soupçonnons que les vibrations du robot entraîne des erreurs qui finissent par diverger (intégration double de l'accélération)
- La cartographie fonctionne en conséquence assez mal étant donné que pour cartographier, le robot a besoin de connaître sa position relative

# Ce qu'il reste à faire 
- Améliorer le positionnement, cela peut se faire à l'aide de capteurs de meilleur qualité mais surtout d'utiliser les étiquettes au sein de l'ANDRA pour se recalibrer. PS: En réalité ce problème n'en est pas vraiment un car de nombreux robots, y compris au TechLab et au Loria, ont déjà résolu ce problème (Spot ou le Unitree GO2 sont plutôt bons). 

- Améliorer l'autonomie du robot, pour l'instant la mission est uniquement d'avancer en ligne droite, à terme l'objectif est que le robot se balade de façon autonome dans l'ensemble des galeries. 

- L'IA est pour l'instant restreinte à un unique type de mur (GER), en effet, la segmentation des fissures est un problème difficile, nous n'avons donc entrainé notre IA sur un unique type de mur, il faudrait le faire pour chacun de la galerie (force à vous) 

# Détails sur le travail réalisé

- Nous avons utilisé ROS2 humble pour le projet, nous vous conseillons FORTEMENT de soit garder cette version soit s'assurer que la version de l'OS (Ubuntu 20.04 par exemple) est bien compatible avec la distribution ROS choisie. 

- Nous utilisons YOLOv11 pour la détection d'images, nous vous conseillons de garder cela car c'est à la fois facile d'utilisation (pas besoin de s'embêter 
avec pytorch ou CUDA tout est géré par défaut) et permet d'obtenir des résultats corrects. Pour la segmentation, utiliser des masques sur LabelStudio, vous utiliserez ensuite un script permettant de transformer les masques en bouding box compréhensibles par YOLO, en réalité assez peu d'informations est perdue, ce n'est pas dérangeant. 

- Pour le LIDAR, nous avons utilisé le SDK officiel : https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble, qui n'est malheuresement pas mis à jour et avec une documentation plus que minimaliste. Ce qu'il faut savoir c'est que la commande ``ros2 launch ydlidar_ros2_driver ydlidar_launch.py'' publie sur le topic /scan les données du LIDAR. Le LIDAR peut ne pas être correctement orienté (pas dans la même direction que l'avant du robot), il faut donc faire une rotation, pour cela dans un autre terminal vous pouvez lancer la commande ```ros2 run tf2_ros static_transform_publisher 0 0 0 1.57 0 0 base_link laser_frame'''.

- Pour la ZED2, nous avons encore une fois utilisé le SDK officiel : https://www.stereolabs.com/docs/ros2 ; celui la est bien mieux documenté, vous n'aurez normalement pas de problème. Il faut cependant noter que lorsque vous arrêtez votre programme, il peut arriver que le noeud ROS de la ZED2 ne s'arrête pas correctement, il faut donc aller le fermer "manuellement", ChatGPT est votre ami si vous ne savez pas comment faire. 

- Pour les roues du robot, le CAN est configuré sur la "mauvaise interface", il faut donc lancer la commande : ``` sudo ip link set can1 up type can bitrate 500000''' avant de pouvoir recevoir les informations. Il existe un repo pour récupérer les données des roues mais nous ne sommes pas sûr duquel nous avons utilisé, probablement celui la : https://github.com/agilexrobotics/scout_ros2. Dans tous les cas, vous pouvez regarder le fichier de lancement qui configure tout si vous voulez plus d'informations. Les informations des roues sont publiés sur le topic /odom_robot

- Pour la PTZ, nous utilisons un protocole qui s'appelle RTSP permettant de récupérer le flux vidéo. Nous avons ensuite développer des noeuds ROS qui utilisent l'IA que nous avons entrainé pour détecter les fissures. Un noeud appelé ``publisher'' envoie une frame du flux toutes les 10 secondes sur le topic (à remplir) tandis que le noeud ``subscriber'' fait le travail d'analyse. L'IA étant précise et possèdant un nombre de couches important, l'analyse d'une image met environ 5 secondes, ce qui n'est pas particulièrement problématique si l'on considère que le robot va travailler de nuit. 

# Démonstration de notre travail

- Pour faire une démo de ce que nous avons fait, vous avez besoin de plusieurs choses : un accès SSH à la Jetson, un PC avec de préférence une carte graphique NVIDIA (je n'ai pas testé sur d'autres je ne sais pas si ça marche quand même) sous Linux avec un environnement X11 et docker-compose d'installé (si vous n'en n'avez pas ou que vous ne comprenez pas ce que j'ai écris vous devriez pouvoir utiliser un des ordis du techlab sans trop de soucis) et de télécharger l'archive contenant le docker (save_docker.zip sur le drive). 

- Connectez vous en SSH à la Jetson (techlab@192.168.50.100 si vous êtes sur Techlab_WIFI, mdp : depinfonancy), et lancez la commande décrite dans la section sur les roues du robot (cela permet de s'assurer que les informations des roues sont sur la bonne interface (demander à Antoine si besoin)). Ouvrez un deuxième terminal SSH et lancez la commande pour la rotation du LIDAR. Enfin, allez dans le dossier ``ros_launcher'' et lancez la commande ```ros2 launch navigation_stack.launch.py```. Normalement, beaucoup d'informations vont s'afficher provenant de tous les noeuds décrits précédemment, si vous avez une erreur lisez bien les logs et chercher sur Google/ChatGPT. La commande précédente lancera le SLAM (lire le rapport).

- Pour observer les avancées de votre robot, sur votre PC sous linux, lancer le fichier de lancement ```./launch.sh''' se trouvant dans l'archive mentionnée précemment. Normalement, l'image docker va se télécharger et vous ouvrir une fenêtre RViz2 (outil de visualisation). Faites ensuite, file -> Open Config et sélectionnez la configuration se trouvant dans le dossier /tmp/config.rviz. Si vous êtes sur le même réseau que le robot (c'est normalement le cas si vous avez réussi à vous connecter en SSH) vous devriez voir apparaitre votre petit robot avec la carte se dessinant autour de ce dernier. 

- Pour lancer AMCL avec une carte prédéfinnie, utilisez la commande suivante : ```ros2 launch navigation_stack.launch.py use_slam:=false use_amcl:=true map_path:=wall_techlab.yaml''' (où wall_techlab.yaml correspond à vos métadonnées de votre carte).

# Contact 

En cas de problèmes que vous n'arrivez pas à résoudre, contactez moi par mail (eliott.leboeuf@gmail.com) ou Messenger, j'essayerai de vous répondre, bonne chance ! 
