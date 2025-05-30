#!/bin/bash

# Helper message function
print_help() {
    echo "Usage: $(basename "$0") [--coeff-dir DIR] [additional arguments]"
    echo ""
    echo "Options:"
    echo "  -d, --coeff-dir DIR   Specify the calibration coefficients directory (default: may2025)"
    echo "  -h, --help            Show this help message and exit"
    echo ""
    echo "Any additional arguments will be passed to compile_calibration.py."
}

# Show help if requested
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    print_help
    exit 0
fi

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default value for coeff_dir
COEFF_DIR="calibrations/may2025"

# Parse arguments for --coeff-dir and -o (output file)
ARGS=()
OUTPUT_FILE="calibration_upload_packets.bin"
while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--coeff-dir)
            COEFF_DIR="calibrations/$2"
            shift 2
            ;;
        -o)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        -*)
            echo "Error: Unrecognized option '$1'"
            print_help
            exit 1
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done
if [[ -n "$OUTPUT_FILE" ]]; then
    ARGS+=("-o" "$OUTPUT_FILE")
fi

# Check if the Python script exists
if [[ ! -f "$SCRIPT_DIR/compile_calibration.py" ]]; then
    echo "Error: compile_calibration.py not found in $SCRIPT_DIR."
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
    export PYTHONPATH="$TARGET_PATH:$PYTHONPATH"
fi

# Build the command to run the Python script
CMD=(python3 "$SCRIPT_DIR/compile_calibration.py")
if [[ -n "$COEFF_DIR" ]]; then
    CMD+=("$COEFF_DIR/temp.coeff")
    CMD+=("$COEFF_DIR/moisture.coeff")
fi
CMD+=("${ARGS[@]}")

# Run the Python script with the arguments
"${CMD[@]}"