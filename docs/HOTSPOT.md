# Guide d'Utilisation Réseau : Techlab-wifi VS Hotspot

Ce document explique comment se connecter au robot (Orin2) selon que vous êtes au Techlab (accès Internet) ou à l'extérieur (mode autonome).

## Au Techlab (Mode Client)

**Le contexte :** Le robot est allumé dans les locaux. Il détecte le WiFi du Techlab et s'y connecte automatiquement car c'est sa priorité haute.

1. **Votre Ordi :** Restez connecté au Techlab-wifi.
2. **Le Robot :** Il est connecté au même réseau que vous (IP type `192.168.40.xxx`).
3. **Connexion :**
```bash
ssh techlab@192.168.40.100
# OU 
ssh techlab@orin2.local
```

---

## Sur le Terrain (Mode Hotspot)

**Le contexte :** Vous allumez le robot dehors. Il ne trouve pas le WiFi du Techlab. Il bascule automatiquement sur sa connexion de secours et crée son propre réseau WiFi.

1. **Votre Ordi :** Ouvrez vos réseaux WiFi et chercher `JetsonWIFI`.
3. **Connectez-vous** Mdp : `depinfonancy`.
4. **Connexion SSH :**
```bash
ssh techlab@orin2.local

```
**Important :** N'utilisez pas l'IP `192.168.40.100` ici. En mode Hotspot, le robot prend souvent l'IP `10.42.0.1`. L'adresse `.local` gère ce changement pour vous automatiquement.

---

## Commandes utiles 

Si vous avez besoin de modifier la configuration ou de forcer un mode.

| Action | Commande |
| --- | --- |
| **Voir l'état réseau** | `nmcli con show` |
| **Forcer le mode Hotspot** | `sudo nmcli con up HotspotRobot` |
| **Eteindre le Hotspot** | `sudo nmcli con down HotspotRobot` |
| **Forcer le mode Techlab-wifi** | `sudo nmcli con up "Techlab-WIFI"` |
| **Désactiver le WiFi** | `sudo nmcli con down HotspotRobot` |