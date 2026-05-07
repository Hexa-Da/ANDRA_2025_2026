#!/bin/bash
# Configuration réseau pour la caméra PTZ
#
# Variables d'environnement (avec défauts adaptés à la Jetson Orin du TechLab) :
#   PTZ_INTERFACE       Interface Ethernet vers la PTZ        (def: enP8p1s0)
#   PTZ_WIFI_INTERFACE  Interface WiFi à nettoyer côté routes (def: wlP1p1s0)
#   PTZ_IP_ADDRESS      IP statique à poser sur l'Ethernet    (def: 192.168.5.100)
#   PTZ_NETMASK         Masque CIDR                           (def: 24)
#   PTZ_NETWORK         Réseau de la caméra                   (def: 192.168.5.0/24)

set -eo pipefail

INTERFACE="${PTZ_INTERFACE:-enP8p1s0}"
IP_ADDRESS="${PTZ_IP_ADDRESS:-192.168.5.100}"
NETMASK="${PTZ_NETMASK:-24}"
NETWORK="${PTZ_NETWORK:-192.168.5.0/24}"
WIFI_INTERFACE="${PTZ_WIFI_INTERFACE:-wlP1p1s0}"

# Attendre que l'interface soit disponible
for i in {1..10}; do
    if ip link show "$INTERFACE" &>/dev/null; then
        break
    fi
    sleep 1
    if [ $i -eq 10 ]; then
        echo "Interface $INTERFACE non trouvée"
        exit 1
    fi
done

# Supprimer l'IP existante si présente (évite les erreurs)
ip addr del "$IP_ADDRESS/$NETMASK" dev "$INTERFACE" 2>/dev/null

# Ajouter l'adresse IP statique
ip addr add "$IP_ADDRESS/$NETMASK" dev "$INTERFACE"

# S'assurer que l'interface est UP
ip link set "$INTERFACE" up

# Supprimer la route WiFi conflictuelle vers le réseau 192.168.5.0/24
ip route del "$NETWORK" dev "$WIFI_INTERFACE" 2>/dev/null

# Vérifier que la route passe bien par Ethernet
if ! ip route show "$NETWORK" | grep -q "$INTERFACE"; then
    echo "Attention: La route vers $NETWORK ne passe pas par $INTERFACE"
fi

echo "Configuration réseau PTZ terminée: $IP_ADDRESS/$NETMASK sur $INTERFACE"