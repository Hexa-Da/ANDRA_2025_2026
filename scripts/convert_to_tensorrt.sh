#!/bin/bash
# Script pour convertir best.pt en best.engine (TensorRT) pour Jetson
# Usage: ./scripts/convert_to_tensorrt.sh
# 
# IMPORTANT: Cette conversion ne modifie PAS le fichier best.pt original
# Elle crée un nouveau fichier best.engine à côté

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="$PROJECT_DIR/ros2_ws/models"
MODEL_PT="$MODEL_DIR/best.pt"
MODEL_ENGINE="$MODEL_DIR/best.engine"

echo "🔄 Conversion de best.pt vers TensorRT (.engine)"
echo "📁 Modèle source: $MODEL_PT"
echo "📁 Modèle cible: $MODEL_ENGINE"
echo ""

# Vérifier que best.pt existe
if [ ! -f "$MODEL_PT" ]; then
    echo "❌ Erreur: best.pt non trouvé dans $MODEL_DIR"
    exit 1
fi

# Vérifier que le modèle n'est pas déjà converti
if [ -f "$MODEL_ENGINE" ]; then
    echo "⚠️  best.engine existe déjà!"
    read -p "Voulez-vous le regénérer? (o/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Oo]$ ]]; then
        echo "Annulé."
        exit 0
    fi
    rm -f "$MODEL_ENGINE"
fi

echo "🚀 Lancement de la conversion dans Docker..."
echo "   ⏳ Cela peut prendre 5-15 minutes selon la taille du modèle"
echo ""

# Utiliser le conteneur Docker pour la conversion
sudo docker run --rm \
    --runtime nvidia \
    -v "$MODEL_DIR:/models" \
    -w /models \
    ros2-humble-pytorch-jetson:r36.2.0 \
    python3 -c "
from ultralytics import YOLO
import torch
import sys

print('📥 Chargement du modèle best.pt...')
model = YOLO('/models/best.pt')

print('🔄 Conversion vers TensorRT (.engine)...')
print('   Paramètres: device=0, half=True, imgsz=640')
try:
    model.export(
        format='engine',
        device=0,
        half=True,
        imgsz=640,
        verbose=True
    )
    print('✅ Conversion réussie!')
    print('📁 Fichier créé: /models/best.engine')
    # Forcer la synchronisation pour s'assurer que le fichier est écrit
    import os
    os.sync()
except Exception as e:
    print(f'❌ Erreur lors de la conversion: {e}')
    sys.exit(1)
" || true  # Ignorer le code de retour si Python crash après l'export

# Vérifier si le fichier a été créé avec succès
# (même si Python crash après l'export, le fichier peut être valide)
if [ -f "$MODEL_ENGINE" ]; then
    FILE_SIZE=$(stat -f%z "$MODEL_ENGINE" 2>/dev/null || stat -c%s "$MODEL_ENGINE" 2>/dev/null || echo "0")
    # Vérifier que le fichier fait au moins 1 MB (taille minimale raisonnable pour un modèle TensorRT)
    if [ "$FILE_SIZE" -gt 1048576 ]; then
        echo ""
        echo "✅ Conversion terminée avec succès!"
        echo "📁 Fichier créé: $MODEL_ENGINE ($(du -h "$MODEL_ENGINE" | cut -f1))"
        echo ""
        echo "💡 Le fichier best.pt original est intact"
        echo "💡 image_subscriber.py utilisera automatiquement best.engine s'il existe"
        echo ""
        echo "⚠️  Note: Un crash Python après l'export est normal et n'affecte pas le fichier créé"
    else
        echo ""
        echo "⚠️  Le fichier best.engine existe mais semble trop petit ($FILE_SIZE bytes)"
        echo "   La conversion a peut-être échoué. Vérifiez les logs ci-dessus."
        exit 1
    fi
else
    echo ""
    echo "❌ Erreur: Le fichier best.engine n'a pas été créé"
    echo "   Vérifiez les logs ci-dessus pour plus de détails"
    exit 1
fi
