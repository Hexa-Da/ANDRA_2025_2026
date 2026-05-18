# 💾 Procédure de Sauvegarde Complète du Robot (Image d'Usine)

Ce document décrit la procédure permettant de réaliser une image de sauvegarde brute compressée du disque NVMe interne du robot (`/dev/nvme0n1`) depuis un ordinateur local.

## Dépôt Git : dossier `backup/`

À la racine du projet, le dossier **`backup/`** sert à versionner **`backup_robot_usine.img.zst.sha256`** (empreinte SHA256 de l’image). Le fichier **`backup_robot_usine.img.zst`** lui-même n’est en principe **pas** dans le dépôt (plusieurs centaines de gigaoctets). La copie de travail de l’archive pour l’équipe est fournie sur une **clé USB jointe au robot** (transmission projet à projet) ; vous pouvez en dupliquer le contenu ailleurs (disque, NAS) pour vos sauvegardes personnelles. Lors d’une nouvelle sauvegarde, mettez à jour le fichier `.sha256` dans `backup/` pour refléter la nouvelle image, afin que l’équipe puisse vérifier toute copie avec `sha256sum -c`.

## 🛠️ Prérequis

1. **Matériel :** Un câble USB-C/USB-C (ou USB-A/USB-C) de bonne qualité connecté entre le robot et votre ordinateur. Votre ordinateur doit disposer d'un port USB performant.
2. **Logiciels sur le Robot :** Les paquets `zstd`, `pv` et `ncdu` doivent être installés.  
   ```bash  
   sudo apt update && sudo apt install -y zstd pv ncdu
   ```
3. **Réseau :** Le câble USB-C active le protocole *USB Device Mode* de la carte Nvidia Jetson. Le robot est accessible de manière dédiée et ultra-rapide à l'adresse IP statique **192.168.55.1**.


## **🚀 Étape 1 : Nettoyage et Préparation du Disque (Sur le Robot)**

Avant de copier le disque, il est indispensable de le nettoyer pour réduire la taille finale de l'image et d'effacer les résidus de données pour maximiser la compression. Connectez-vous en SSH classique au robot.

### **1.1 Purge des fichiers inutiles**

Exécutez les commandes suivantes pour nettoyer les caches et les journaux :

```bash
# Supprime les dépendances APT obsolètes  
sudo apt-get autoremove --purge  
sudo apt-get clean

# Limite la taille des logs système à 100 Mo  
sudo journalctl --vacuum-size=100M

# Vide le cache de l'utilisateur courant  
rm -rf ~/.cache/*
```

*(Optionnel)* Lancez `sudo ncdu` pour traquer manuellement d'éventuels gros fichiers de données obsolètes (enregistrements de capteurs, .bag, etc.).

### **1.2 Écriture de zéros sur l'espace vide**

`dd` copie l'intégralité du disque (930 Go), y compris l'espace invisible. Pour que l'algorithme de compression réduise cet espace vide à néant, nous devons le remplir de zéros binaires :

```bash
# Crée un fichier de zéros jusqu'à saturation du disque  
sudo dd if=/dev/zero of=/zero_file bs=1M status=progress

# Supprime immédiatement ce fichier temporaire  
sudo rm /zero_file
```


## **📦 Étape 2 : Lancement de la Sauvegarde (Depuis votre PC local)**

Ouvrez un terminal **sur votre propre ordinateur** (pas en SSH sur le robot). Placez-vous dans le dossier où vous souhaitez stocker la sauvegarde.  
Exécutez la commande suivante, en remplacant MOT_DE_PASSE_ROBOT par le mot de passe du robot.

```bash
ssh -c aes128-gcm@openssh.com techlab@192.168.55.1 "echo 'MOT_DE_PASSE_ROBOT' | sudo -S dd if=/dev/nvme0n1 bs=4M 2>/dev/null | pv -fs 930G | zstd -1 -T0" > backup_robot_usine.img.zst
```

### **🔍 Détails de cette commande :**

* `ssh -c aes128-gcm@openssh.com` : Utilise un chiffrement SSH plus léger pour ne pas brider le processeur.  
* `sudo -S` : Reçoit le mot de passe via l'entrée standard pour éviter le blocage du terminal.  
* `dd if=/dev/nvme0n1 bs=4M` : Lit le disque NVMe par blocs de 4 Mo.  
* `pv -fs 930G` : Génère une barre de progression en temps réel basée sur la taille totale de 930 Go (optionnel).
* `zstd -1 -T0` : Compresse les données à la volée en utilisant **tous** les cœurs du processeur du robot.  
* `> backup_robot_usine.img.zst` : Enregistre le résultat dans un fichier compressé .zst sur votre machine.

## **🔒 Étape 3 : Sécurisation de l'Image (Depuis votre PC local)**

Une fois la barre de progression arrivée à 100%, générez une empreinte numérique (checksum) pour garantir que l'image ne sera pas corrompue au fil des ans :

```bash
sha256sum backup_robot_usine.img.zst > backup_robot_usine.img.zst.sha256
```

Pour vérifier l'intégrité plus tard avant un flashage, exécutez :

```bash
sha256sum -c backup_robot_usine.img.zst.sha256  
```


# 🤖 PROCÉDURE DE RESTAURATION DU ROBOT (Méthode Live USB) 

**Fichier de sauvegarde :** `backup_robot_usine.img.zst` 

---

## ⚠️ AVERTISSEMENT   
Cette procédure va **écraser intégralement** le disque NVMe interne du robot.


## 🛠️ MATÉRIEL REQUIS  
* **Écran :** Un écran DisplayPort/USB-C branché à la carte du robot.  
* **Clavier :** Un clavier USB branché au robot.  
* **Clé USB Système :** Une clé USB amorçable (Live USB) contenant un système d'exploitation Linux compatible ARM64.  
* **Clé USB Sauvegarde :** Le fichier `backup_robot_usine.img.zst` stocké sur une autre clé USB.


## 🚀 ÉTAPES DE RESTAURATION

### 1. Démarrer sur la clé USB (Système Live)  
* Branchez la clé USB amorçable, le clavier et l'écran sur le robot éteint.  
* Allumez le robot. Appuyez frénétiquement sur la touche **Échap (ESC)** ou **F11** de votre clavier pour accéder au menu de démarrage (UEFI/BIOS de la carte mère Nvidia).  
* Sélectionnez votre clé USB pour démarrer dessus.

### 2. Identification du disque interne   
Une fois sur le bureau Linux de la clé USB, ouvrez un terminal et tapez la commande suivante:  
```bash  
lsblk
```

Repérez le disque NVMe interne (il s'appellera généralement /dev/nvme0n1). Assurez-vous qu'aucune partition de ce disque n'est montée (il ne doit y avoir aucun point de montage / ou /boot en face de ce disque).

### 3. Lancement du flashage

* Branchez le support contenant la sauvegarde.

* Ouvrez un terminal dans le dossier où se trouve le fichier backup_robot_usine.img.zst.

* Lancez la commande suivante:

```bash
sudo zstdcat backup_robot_usine.img.zst | sudo dd of=/dev/nvme0n1 bs=4M status=progress
```

(Note : La première partie de la commande extrait l'image , tandis que la seconde écrit directement sur le disque NVMe par blocs de 4 Mo ).

### 4. Finalisation

* Une fois le transfert terminé à 100%, tapez la commande `sync` pour vider les caches d'écriture.

* Éteignez le robot avec la commande : `sudo poweroff`.

* Retirez toutes les clés USB.

* Rallumez le robot : il démarrera sur son disque NVMe interne fraîchement restauré !  
