# Détection YOLO (fissures) et TensorRT

Ce document décrit le nœud `image_subscriber`, le choix du modèle (PyTorch vs TensorRT), les chemins de fichiers et les variables d’environnement. Le comportement opérationnel correspond au code dans `ros2_ws/src/image_transfer/image_transfer/image_subscriber.py`.

## Ce qui a été mis en place (optimisation Jetson)

Sur **Jetson Orin**, la détection intensive ne tourne pas dans le même environnement que toute la stack ROS : on a isolé l’inférence YOLO dans un **conteneur Docker** avec **CUDA**, tout en gardant le même workspace monté pour les modèles et les résultats.

1. **Image Docker dédiée** — `docker/Dockerfile.jetson`, basée sur **`dustynv/l4t-pytorch:r36.2.0`**, pour un support GPU cohérent avec la plateforme NVIDIA. L’image vise à embarquer **ROS 2 Humble**, **PyTorch** avec CUDA, et les dépendances Python utiles à la chaîne **Ultralytics + OpenCV**, avec **NumPy < 2** pour rester compatible avec **cv_bridge**. L’objectif : un traitement des images bien plus rapide !

2. **Accélération TensorRT** — Le script **`scripts/convert_to_tensorrt.sh`** convertit **`ros2_ws/models/best.pt`** en **`best.engine`**. L’export s’exécute dans un conteneur (même famille d’images Jetson) avec précision **half**, **GPU** et taille **640×640**, afin d’aligner l’export et l’inférence PyTorch. Sur Jetson, l’inférence TensorRT est en général **nettement plus rapide** que PyTorch.

3. **Scripts de lancement** — **`scripts/image_subscriber_gpu.sh`** est le point d’entrée : il délègue à **`docker/launch_jetson.sh`**, qui monte **`ros2_ws`**, utilise le **réseau host** pour parler aux autres nœuds ROS2, vérifie la compilation du workspace, la lisibilité des modèles, et installe ou vérifie au besoin ROS 2 / Ultralytics dans le conteneur avant d'exécuter **`ros2 run image_transfer image_subscriber`**.

4. **Code Python** — **`image_subscriber.py`** choisit en priorité **`best.engine`** si le GPU est dispo et qu’on n’impose pas PyTorch (`FORCE_PYTORCH`). Sinon **`best.pt`** en **PyTorch sur GPU** (`device=0`, FP16) si CUDA est disponible dès le départ. En revanche, un **repli après échec TensorRT** (mémoire, chargement ou inférence) recharge **`best.pt`** en **CPU** pour stabiliser ; le code peut **retenter le GPU** pour PyTorch tous les **`GPU_RETRY_INTERVAL`** images si la mémoire le permet.

Les sections suivantes détaillent le fonctionnement du nœud et les commandes utiles.

## Rôle du nœud `image_subscriber`

- **Entrée** : `sensor_msgs/Image` sur **`/photo_topic`** (images publiées par `image_publisher` ou `video_publisher`).
- **Modèle** : YOLO Ultralytics en **segmentation** (`task='segment'`).
- **Sorties** :
  - Images annotées (si détection) dans **`ros2_ws/images_detectees/`**
  - **`geometry_msgs/Point`** sur **`position_detectee`** : position **odom** (`x`, `y`, `z`) au moment de la détection (topic **`/odometry/filtered`**). Le nœud **`report_fissures`** convertit ces coordonnées en repère **map** via TF (`map` ← `odom`, mis à jour par AMCL ou le 2D Pose Estimate).

## Fichiers modèle (`best.pt` / `best.engine`)

Le nœud cherche **`best.engine`** (TensorRT) puis **`best.pt`** (PyTorch), dans cet ordre. Si aucun des deux fichiers n’est trouvé, le nœud lève une erreur au chargement du modèle.

## Choix TensorRT vs PyTorch

1. **GPU CUDA** : si `torch.cuda.is_available()` est faux, l’inférence utilise le **CPU**
2. **`FORCE_PYTORCH=1`** : désactive TensorRT même si `best.engine` est présent et que le GPU est disponible ; seul le flux **PyTorch** est utilisé.
3. **Sinon** : si **`best.engine` existe**, est lisible, et que le GPU est dispo, le nœud utilise **TensorRT** (`best.engine`).
4. **Sinon** : utilisation de **`best.pt`** (PyTorch).

### Chargement du modèle

Le modèle est chargé **au premier callback** sur `/photo_topic`.

## Variables d’environnement

| Variable | Effet |
|----------|--------|
| **`FORCE_PYTORCH=1`** | Force l’usage de `best.pt` ; ignore `best.engine` même sur GPU. |
| **`GPU_RETRY_INTERVAL`** | Nombre d’images entre deux tentatives de repasser sur **GPU** après un repli **CPU** en PyTorch (défaut : **`50`**). |
| **`MIN_GPU_FREE_GB`** | Mémoire CUDA libre minimale avant TensorRT (défaut : **`3.0`**). Un warmup teste l’engine au chargement ; en dessous de ~3 Go avec ZED/AMCL actifs, fallback CPU. |

### Mémoire GPU et bascules (mai 2026)

Sur **Jetson Orin** avec ZED + AMCL/SLAM actifs, TensorRT peut réussir vers **~3,3 Go** CUDA libres et échouer (NvMap) vers **~2,4 Go**. Le nœud :

1. Mesure la mémoire avec **`torch.cuda.mem_get_info`** 
2. Refuse TensorRT si la mémoire libre est **< `MIN_GPU_FREE_GB`** (défaut **3.0**).
3. Exécute un **warmup** (première inférence factice 640×640) juste après le chargement de `best.engine` — c’est là qu’Ultralytics désérialise l’engine ; en cas d’échec, bascule **`best.pt` sur CPU** et désactive TensorRT pour la session (`_tensorrt_disabled`).
4. Libère proprement le modèle (`_unload_model` + `gc` + `empty_cache`) avant tout changement de backend ou à l’arrêt du nœud (évite segfault NvMap).

D’autres erreurs d’inférence en cours d’exécution déclenchent aussi TensorRT → PyTorch CPU, ou PyTorch GPU → CPU selon le cas ; repli GPU PyTorch retenté tous les **`GPU_RETRY_INTERVAL`** images.

## Conversion `best.pt` → `best.engine`

```bash
./scripts/convert_to_tensorrt.sh
```

Référence : `ros2_ws/models/best.pt` → export **`engine`**, `device=0`, **`half=True`**, **`imgsz=640`** (aligné avec l’inférence PyTorch GPU). Le fichier sort dans le même dossier : `ros2_ws/models/best.engine`.

## Lancer la détection

**Sur le robot** : depuis la racine du projet, après compilation de `ros2_ws` et stack ROS principale déjà lancée si besoin :

```bash
./scripts/image_subscriber_gpu.sh
```

Ce script appelle **`docker/launch_jetson.sh`** : montage de `ros2_ws`, réseau host, puis `ros2 run image_transfer image_subscriber`.
