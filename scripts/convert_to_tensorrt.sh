#!/bin/bash
# Convertit best.pt en best.engine (TensorRT) pour Jetson
# Usage: ./scripts/convert_to_tensorrt.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="$PROJECT_DIR/ros2_ws/models"
MODEL_PT="$MODEL_DIR/best.pt"
MODEL_ENGINE="$MODEL_DIR/best.engine"

echo "Conversion best.pt -> TensorRT (.engine)"
echo "Source: $MODEL_PT"

if [ ! -f "$MODEL_PT" ]; then
    echo "[ERROR] best.pt non trouve dans $MODEL_DIR"
    exit 1
fi

if [ -f "$MODEL_ENGINE" ]; then
    read -p "best.engine existe deja. Regenerer? (o/N): " -n 1 -r
    echo
    [[ ! $REPLY =~ ^[Oo]$ ]] && exit 0
    rm -f "$MODEL_ENGINE"
fi

echo "Conversion en cours (5-15 min)..."

sudo docker run --rm \
    --runtime nvidia \
    -v "$MODEL_DIR:/models" \
    -w /models \
    ros2-humble-pytorch-jetson:r36.2.0 \
    python3 -c "
from ultralytics import YOLO
import sys, os

model = YOLO('/models/best.pt')
try:
    model.export(format='engine', device=0, half=True, imgsz=640, verbose=True)
    os.sync()
except Exception as e:
    print(f'[ERROR] {e}')
    sys.exit(1)
" || true

# Verification du fichier genere
if [ -f "$MODEL_ENGINE" ]; then
    FILE_SIZE=$(stat -f%z "$MODEL_ENGINE" 2>/dev/null || stat -c%s "$MODEL_ENGINE" 2>/dev/null || echo "0")
    if [ "$FILE_SIZE" -gt 1048576 ]; then
        echo "[OK] $MODEL_ENGINE ($(du -h "$MODEL_ENGINE" | cut -f1))"
    else
        echo "[ERROR] Fichier trop petit ($FILE_SIZE bytes)"
        exit 1
    fi
else
    echo "[ERROR] best.engine non cree"
    exit 1
fi
