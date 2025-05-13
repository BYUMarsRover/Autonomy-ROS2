#!/bin/bash

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the Python script exists
if [[ ! -f "$SCRIPT_DIR/compile_routine.py" ]]; then
    echo "Error: compile_routine.py not found in $SCRIPT_DIR."
    exit 1
fi

# Define the root of the Autonomy-ROS2 repository
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

# Define the directory to add to PYTHONPATH
TARGET_PATH="$REPO_ROOT/src"

# Check if the directory is already in PYTHONPATH
if [[ ":$PYTHONPATH:" != *":$TARGET_PATH:"* ]]; then
    # Append the directory to PYTHONPATH
    export PYTHONPATH="$TARGET_PATH:$PYTHONPATH"
    echo "Added $TARGET_PATH to PYTHONPATH."
else
    echo "$TARGET_PATH is already in PYTHONPATH."
fi

# Run the Python script with remaining command-line arguments
python3 "$SCRIPT_DIR/compile_routine.py" $@