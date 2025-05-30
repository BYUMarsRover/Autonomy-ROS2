#!/bin/bash

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if the Python script exists
if [[ ! -f "$SCRIPT_DIR/compile_routine.py" ]]; then
    echo "Error: compile_routine.py not found in $SCRIPT_DIR."
    exit 1
fi

# Find the path to 'mars_ws/src' by searching upwards from SCRIPT_DIR
SEARCH_DIR="$SCRIPT_DIR"
while [[ "$SEARCH_DIR" != "/" && ! -d "$SEARCH_DIR/mars_ws/src" ]]; do
    SEARCH_DIR="$(dirname "$SEARCH_DIR")"
done

if [[ ! -d "$SEARCH_DIR/mars_ws/src" ]]; then
    echo "Error: Could not find 'mars_ws/src' in any parent directory of $SCRIPT_DIR."
    exit 1
fi

# Define the directory to add to PYTHONPATH
TARGET_PATH="$SEARCH_DIR/mars_ws/src"

# Check if the directory is already in PYTHONPATH
if [[ -z "$PYTHONPATH" ]]; then
    export PYTHONPATH="$TARGET_PATH"
elif [[ ":$PYTHONPATH:" != *":$TARGET_PATH:"* ]]; then
    # Append the directory to PYTHONPATH
    export PYTHONPATH="$TARGET_PATH:$PYTHONPATH"
fi

# Run the Python script with remaining command-line arguments
python3 "$SCRIPT_DIR/compile_routine.py" $@