#!/bin/bash
# Script de test à exécuter quand le robot sera disponible

echo "=== Test du système complet ==="

# 1. Vérifier les noeuds
echo "1. Vérification des noeuds..."
ros2 node list | grep -E "(ydlidar|zed|scout|ekf|slam)" || echo "⚠ Noeuds non trouvés"

# 2. Vérifier les topics
echo "2. Vérification des topics..."
ros2 topic list | grep -E "(scan|odom|map|zed)" || echo "⚠ Topics non trouvés"

# 3. Vérifier les TF
echo "3. Vérification des transformations..."
ros2 run tf2_ros tf2_echo map base_link --once || echo "⚠ TF non disponibles"

# 4. Vérifier la fréquence des topics
echo "4. Fréquences des topics..."
ros2 topic hz /scan --window 5 &
ros2 topic hz /odom_robot --window 5 &
ros2 topic hz /odometry/filtered --window 5 &
wait

echo "=== Tests terminés ==="
