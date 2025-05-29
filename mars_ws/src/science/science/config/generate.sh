#!/bin/bash

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# # Define the root of the Autonomy-ROS2 repository
# REPO_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

# # Define the directory to add to PYTHONPATH
# TARGET_PATH="$REPO_ROOT/src"

# # Check if the directory is already in PYTHONPATH
# if [[ -z "$PYTHONPATH" ]]; then
#     export PYTHONPATH="$TARGET_PATH"
# elif [[ ":$PYTHONPATH:" != *":$TARGET_PATH:"* ]]; then
#     # Append the directory to PYTHONPATH
#     export PYTHONPATH="$TARGET_PATH:$PYTHONPATH"
# fi

# Parse -o flag for output filename, default to ./eeprom_upload.bin
OUTPUT_FILE="eeprom_upload.bin"
while [[ $# -gt 0 ]]; do
    case "$1" in
        -o)
            shift
            OUTPUT_FILE="$1"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# Find all immediate subdirectories with a generate.sh file and produce
> "$OUTPUT_FILE"
for dir in "${SCRIPT_DIR}"/*/; do
    if [[ -f "${dir}generate.sh" ]]; then
        DIR_NAME=$(basename "$dir")
        TEMP_FILE="${DIR_NAME}.bin"
        echo -e "\033[1;33mGenerating in ${DIR_NAME}...\033[0m"
        bash "${dir}generate.sh" -o "../${TEMP_FILE}"
        cat $TEMP_FILE >> $OUTPUT_FILE
        rm $TEMP_FILE
    fi
done

echo -e "\033[1;33mComplete EEPROM upload binary written to ${OUTPUT_FILE}\033[0m"
cp ${OUTPUT_FILE} ../processing_gui/packets/eeprom_upload.bin
echo "Copied to ../processing_gui/packets/eeprom_upload.bin"
