#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

# Define the input CSV file and output test files
INPUT_CSV="science_module_function_map.csv"
OUT_DIRECTORY="../science_module_arduino/src/command_execution"
FINAL_DIR=$(basename "$OUT_DIRECTORY")
SOURCE_NAME="science_module_functions"
HEADER_OUTPUT="${OUT_DIRECTORY}/${SOURCE_NAME}.h"
IMPLEMENTATION_OUTPUT="${OUT_DIRECTORY}/${SOURCE_NAME}.cpp"
SWITCH_CASE_OUTPUT="${OUT_DIRECTORY}/science_switch.cpp"

# Run the Python scripts with the specified input and output files
echo "Generating function headers..."
python3 build_function_headers.py "$INPUT_CSV" "$HEADER_OUTPUT"

echo "Mantaining function implementations..."
python3 build_function_implementations.py "$INPUT_CSV" "$IMPLEMENTATION_OUTPUT" "${SOURCE_NAME}.h"

echo "Generating switch case file..."
python3 build_switch_case.py "$INPUT_CSV" "$SWITCH_CASE_OUTPUT" --headers "${SOURCE_NAME}.h" "${FINAL_DIR}.h"

echo "All files generated successfully:"
echo " - Function headers: $HEADER_OUTPUT"
echo " - Function implementations: $IMPLEMENTATION_OUTPUT"
echo " - Switch case: $SWITCH_CASE_OUTPUT"