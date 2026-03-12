#!/bin/bash

# sudo apt install inotifywait

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REMOTE_USER="mitch"
REMOTE_HOST="192.168.66.2"
REMOTE_PATH="/home/$REMOTE_USER/R2D2"
LOCAL_PATH="$SCRIPT_DIR/"

# inotifywait --monitor --recursive --quiet --event modify,create,delete,move \
#     --exclude '(\.git|\.cache|build|install|log|\.pio)' \
#     "$SCRIPT_DIR" |
#     while read -r; do
#         echo "Change detected, syncing..."
#         rsync -avz --progress \
#             --exclude='.git/' \
#             --exclude='.cache/' \
#             --exclude='build/' \
#             --exclude='install/' \
#             --exclude='log/' \
#             --exclude='.pio/' \
#             -e "ssh" \
#             "$LOCAL_PATH" \
#             "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}"
#     done

inotifywait --monitor --recursive --quiet --event modify,create,delete,move \
    --exclude '(\.git|\.cache|build|install|log|\.pio)' \
    "$SCRIPT_DIR" |
    while read -r; do
        # drain any additional queued events
        while read -r -t 0.5; do :; done
        echo "Change detected, syncing..."
        rsync -avz --progress \
            --exclude='.git/' \
            --exclude='.cache/' \
            --exclude='build/' \
            --exclude='install/' \
            --exclude='log/' \
            --exclude='.pio/' \
            -e "ssh" \
            "$LOCAL_PATH" \
            "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}"
    done
