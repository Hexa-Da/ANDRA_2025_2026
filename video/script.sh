#!/bin/bash

# Configuration
RTSP_URL='rtsp://admin:admin@192.168.5.163:554/live/av0'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/video_output"
FFMPEG_PATH=$(which ffmpeg)
FFPLAY_PATH=$(which ffplay)

# Paramètres de retouche d'image (identique à image_publisher.py)
BRIGHTNESS=1.4     # Multiplicateur de luminosité (1.0 = normal, >1.0 = plus clair)
CONTRAST=1.4       # Multiplicateur de contraste (1.0 = normal, >1.0 = plus de contraste)
GAMMA=1.2         # Correction gamma (1.0 = normal, <1.0 = plus clair)
ENABLE_ADJUSTMENT=true  # Activer/désactiver les retouches

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Fonction pour construire le filtre vidéo FFmpeg
build_video_filter() {
    if [[ "$ENABLE_ADJUSTMENT" == "true" ]]; then
        # Convertir la luminosité de multiplicateur vers la valeur FFmpeg (-1.0 à 1.0)
        # Formule approximative : brightness_ffmpeg = (brightness - 1.0) / 2.0
        # Pour brightness=3.0 -> (3.0-1.0)/2.0 = 1.0 (maximum)
        # Limiter entre -1.0 et 1.0
        brightness_ffmpeg=$(echo "scale=2; ($BRIGHTNESS - 1.0) / 2.0" | bc)
        brightness_ffmpeg=$(echo "scale=2; if ($brightness_ffmpeg > 1.0) 1.0 else if ($brightness_ffmpeg < -1.0) -1.0 else $brightness_ffmpeg" | bc)
        
        # Construire le filtre eq (equalizer) pour luminosité, contraste et gamma
        echo "eq=brightness=$brightness_ffmpeg:contrast=$CONTRAST:gamma=$GAMMA"
    else
        echo "null"  # Pas de filtre
    fi
}

echo "Press 'r' to start recording, 's' to stop, and 'q' to quit."
if [[ "$ENABLE_ADJUSTMENT" == "true" ]]; then
    echo "Image adjustment enabled: Brightness=$BRIGHTNESS, Contrast=$CONTRAST, Gamma=$GAMMA"
else
    echo "Image adjustment disabled (raw video)"
fi

while true; do
    read -n 1 -s KEY
    if [[ $KEY == "r" ]]; then
        if [[ -n $FFMPEG_PID ]]; then
            echo "Recording already in progress."
        else
            TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
            OUTPUT_FILE="$OUTPUT_DIR/record_$TIMESTAMP.mp4"

            echo "Starting recording: $OUTPUT_FILE"
            
            # Construire la commande FFmpeg avec filtres vidéo si nécessaire
            if [[ "$ENABLE_ADJUSTMENT" == "true" ]]; then
                VIDEO_FILTER=$(build_video_filter)
                # Réencoder la vidéo avec les filtres (H.264 pour compatibilité)
                $FFMPEG_PATH -i "$RTSP_URL" \
                    -vf "$VIDEO_FILTER" \
                    -c:v libx264 -preset medium -crf 23 \
                    -c:a copy \
                    "$OUTPUT_FILE" > /dev/null 2>&1 &
            else
                # Copie directe sans traitement (plus rapide)
                $FFMPEG_PATH -i "$RTSP_URL" -c copy "$OUTPUT_FILE" > /dev/null 2>&1 &
            fi
            
            FFMPEG_PID=$!
            echo "Recording in progress... Press 's' to stop."
        fi
    elif [[ $KEY == "s" ]]; then
        if [[ -n $FFMPEG_PID ]]; then
            echo "Stopping recording..."
            kill -INT $FFMPEG_PID
            wait $FFMPEG_PID
            unset FFMPEG_PID
            echo "Recording saved."
            echo "Press 'q' to quit or 'r' to record again."
        else
            echo "No recording in progress."
            echo "Press 'q' to quit or 'r' to record."
        fi
    elif [[ $KEY == "q" ]]; then
        echo "Exiting the program."
        if [[ -n $FFMPEG_PID ]]; then
            echo "Stopping and saving any ongoing recording..."
            kill -INT $FFMPEG_PID
            wait $FFMPEG_PID
        fi
        exit 0
    else
        echo "Unknown key. Use 'r' to record, 's' to stop, or 'q' to quit."
    fi
done