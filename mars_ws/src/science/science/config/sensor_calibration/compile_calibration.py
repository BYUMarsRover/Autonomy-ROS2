"""
Compiles a config file into a binary file to be uploaded to the science module EEPROM
"""

import csv
import os
import struct
import math
import argparse
from csv import DictReader
import re
from science.science.function_mapping.function_map import ScienceModuleFunctionList as SMFL

from cpypp import py_preprocessor
PYPP = py_preprocessor()

def build_polynomial_string(coeff_list):
    out = ""
    for i in range(len(coeff_list)):
        coeff = coeff_list[i]
        if i == 0:
            out += str(coeff)
        else:
            out += f" + {str(coeff)}*x^{i}"
    return out

def build_calibration_binary(calibration_file_paths):
    # Parse each calibration file into a list of floats
    calibration_data = []
    for i in range(len(calibration_file_paths)):
        file_path = calibration_file_paths[i]
        print(f"Calibrating sensor index {i} from file {file_path}:")
        list = []
        calibration_data.append(list)
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # Flatten and filter out empty strings
                floats = [float(item) for item in row if item.strip()]
                list.extend(floats)
        print(f"\t{build_polynomial_string(list)}")

    # Construct the bytes from each list
    binary_content = bytearray()
    for coeff_list in calibration_data:

        # Write the number of coeffs
        binary_content.extend(len(coeff_list).to_bytes(1, byteorder='little', signed=False))

        # Write each coeff, pack the rest with 0
        for i in range(just.NUM_COEFFS):
            if i < len(coeff_list):
                binary_content.extend(struct.pack('<f', float(coeff_list[i])))
            else:
                binary_content.extend(struct.pack('<f', float(0.0)))
            
    return binary_content

# Example usage
if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Compile calibration files into a binary file for the science module EEPROM.")
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="calibration_upload_packets.bin",
        help="Name of the output binary file (default: calibration_upload_packets.bin)"
    )
    parser.add_argument(
        "sensor_calibration_files",
        nargs="*",
        help="Calibration files for each sensor, in order of sensor index (default: calibrations/may2025/temp.coeff calibrations/may2025/moisture.coeff)"
    )
    args = parser.parse_args()

    if len(args.sensor_calibration_files) != 2:
        raise ValueError(f"Expected 2 calibation files but received {len(args.sensor_calibration_files)}")

    # Ensure paths are relative to the script's directory
    base_dir = os.path.dirname(__file__)
    calibration_file_paths = [os.path.join(base_dir, filename) for filename in args.sensor_calibration_files]

    # Read from memory map
    import science.science.config.mem_map_eval.just as just
    CALIBRATION_DATA_START = just.EEPROM_PTR_TEMP_COEFF_COUNT

    # Substitute tokens and print the result
    eeprom_data = build_calibration_binary(calibration_file_paths)
    packet_stream = SMFL.packetize_eeprom_write(eeprom_data, CALIBRATION_DATA_START, acknowledge=True)

    # Save the binary content to a file
    output_file_path = os.path.join(base_dir, args.output)
    with open(output_file_path, 'wb') as output_file:
        output_file.write(packet_stream)
    print(f"Compiled config binary saved to {output_file_path}")

