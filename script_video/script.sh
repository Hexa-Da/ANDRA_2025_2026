#!/bin/bash

# Configuration
RTSP_URL='rtsp://admin:admin@192.168.5.163:554/live/av0'
OUTPUT_DIR="./videos"
FFMPEG_PATH=$(which ffmpeg)

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

echo "Press 'r' to start recording, 's' to stop, and 'q' to quit."

while true; do
    read -n 1 -s KEY
    if [[ $KEY == "r" ]]; then
        TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
        OUTPUT_FILE="$OUTPUT_DIR/record_$TIMESTAMP.mp4"

        echo "Starting recording: $OUTPUT_FILE"
        $FFMPEG_PATH -i "$RTSP_URL" -c copy "$OUTPUT_FILE" &
        FFMPEG_PID=$!
        echo "Recording in progress... Press 's' to stop."
    elif [[ $KEY == "s" ]]; then
        if [[ -n $FFMPEG_PID ]]; then
            echo "Stopping recording..."
            kill -INT $FFMPEG_PID
            wait $FFMPEG_PID
            unset FFMPEG_PID
            echo "Recording stopped."
        else
            echo "No recording in progress."
        fi
    elif [[ $KEY == "q" ]]; then
        echo "Exiting the program."
        if [[ -n $FFMPEG_PID ]]; then
            echo "Stopping any ongoing recording..."
            kill -INT $FFMPEG_PID
            wait $FFMPEG_PID
        fi
        exit 0
    else
        echo "Unknown key. Use 'r' to record, 's' to stop, or 'q' to quit."
    fi
done


