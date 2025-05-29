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

def build_config_binary(config_file, fields_file):
    # Load the list from the fields file
    fields_dict = []
    with open(fields_file, mode='r') as csv_file:
        reader = DictReader(csv_file)
        fields_dict = list(reader)

    # Read the .routine file and substitute tokens
    config_dict = {}
    with open(config_file, mode='r') as file:
        content = file.read().strip()
        content = re.sub(r'#.*', '', content) # Remove Python-style comments
        for line in content.split('\n'):
            # Remove all new lines and split content into a list of strings
            term, value = map(str.strip, line.split(':', 1))
            config_dict[term] = value

    binary_content = bytearray()
    for field in fields_dict:
        if field['term'] in config_dict:
            # Get the value
            value = config_dict[field['term']]
            datatype = field['datatype']

            print(field['term'], '=', value)

            # Convert the value to its binary representation
            if match := re.match(r'uint(\d+)_t', datatype):
                byte_length = math.ceil(int(match.group(1)) / 8)
                value = int(value, 16) if '0x' in value else int(value)
                binary_content.extend(value.to_bytes(byte_length, byteorder='little', signed=False))
            elif datatype == 'float':
                binary_content.extend(struct.pack('<f', float(value)))
            else:
                raise ValueError(f"Unknown datatype '{datatype}' for token '{content[i]}'.")
        else:
            raise SyntaxError(f"Missing field '{field['term']}' was not found.")
            
    return binary_content

# Example usage
if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Compile config files into a binary file for the science module EEPROM.")
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="config_upload_packets.bin",
        help="Name of the output binary file (default: config_upload_packets.bin)"
    )
    args = parser.parse_args()

    config_file_path = "module.config"
    field_file_path = "fields.csv"

    # Ensure paths are relative to the script's directory
    base_dir = os.path.dirname(__file__)
    config_file_path = os.path.join(base_dir, config_file_path)
    dictionary_path = os.path.join(base_dir, field_file_path)

    # Read from memory map
    try:
        import science.science.config.mem_map_eval.just as just
        EEPROM_CONFIG_START_ADDR = just.EEPROM_CONFIG_START_ADDR
    except:
        raise ValueError("EEPROM_CONFIG_START_ADDR not found in memory map.")

    # Substitute tokens and print the result
    eeprom_data = build_config_binary(config_file_path, dictionary_path)
    packet_stream = SMFL.packetize_eeprom_write(eeprom_data, EEPROM_CONFIG_START_ADDR, acknowledge=True)

    # Save the binary content to a file
    output_file_path = os.path.join(base_dir, args.output)
    with open(output_file_path, 'wb') as output_file:
        output_file.write(packet_stream)
    print(f"Compiled config binary saved to {output_file_path}")

