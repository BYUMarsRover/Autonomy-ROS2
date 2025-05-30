"""
Compiles a routine script into a binary file to be uploaded to the science module EEPROM
"""

import csv
import os
import struct
import math
import argparse
from csv import DictReader
import re
from science.science.function_mapping.function_map import ScienceModuleFunctionList as SMFL

END_OF_ROUTINE_CODE = 0xFF

def build_routine_binary(routine_file, dictionary_file):
    """
    Reads a .routine file and substitutes all tokens with their definitions from the dictionary.csv file.

    Args:
        routine_file (str): Path to the .routine file.
        dictionary_file (str): Path to the dictionary.csv file.

    Returns:
        str: The processed content of the .routine file with tokens substituted.
    """
    # Load the dictionary from the CSV file
    token_dict = {}
    with open(dictionary_file, mode='r') as csv_file:
        reader = DictReader(csv_file)
        dict = list(reader)
        for item in dict:
            token_dict[item['term']] = {
                'value': item['value'],
                'datatype': item['datatype']
            }

    # Read the .routine file and substitute tokens
    with open(routine_file, mode='r') as file:
        content = file.read()
        # Remove Python-style comments
        content = re.sub(r'#.*', '', content)
        # Remove all new lines and split content into a list of strings
        content = content.replace('\n', ' ').split()

    binary_content = bytearray()
    for i in range(len(content)):
        # Attempt to substitute the token with its definition
        if content[i] in token_dict:

            # Get dict info
            value = token_dict[content[i]]['value']
            datatype = token_dict[content[i]]['datatype']

            # Convert the value to its binary representation
            if match := re.match(r'uint(\d+)_t', datatype):
                byte_length = math.ceil(int(match.group(1)) / 8)
                value = int(value, 16) if '0x' in value else int(value)
                binary_content.extend(value.to_bytes(byte_length, byteorder='little', signed=False))
            elif datatype == 'float':
                binary_content.extend(struct.pack('>f', float(value)))
            else:
                raise ValueError(f"Unknown datatype '{datatype}' for token '{content[i]}'.")
        else:
            raise ValueError(f"Token '{content[i]}' not found in dictionary.")
        
    binary_content.append(END_OF_ROUTINE_CODE)
    return binary_content

def build_routine_array_binary(routines_folder, dictionary_file, table_start_addr):
    """
    Reads a folder of .routine files and a dictionary file, compiles each routine into binary,
    and appends them together into a single binary array.
    """

    # Initialize a list of routine binaries
    routine_binary_streams = []

    # Iterate through all .routine files in the folder
    for filename in sorted(os.listdir(routines_folder)):
        if filename.endswith(".routine"):
            print(f'Processing script {filename}...')
            routine_file = os.path.join(routines_folder, filename)
            routine_binary = build_routine_binary(routine_file, dictionary_file)
            routine_binary_streams.append(routine_binary)

    # Concatenate all routine binaries into a single bytearray
    final_binary = bytearray() 

    # Append a single byte with the total number of routines
    final_binary.append(len(routine_binary_streams))

    # Move the pointer to the start of the routine data
    ptr = (table_start_addr + 1) + len(routine_binary_streams) * 2

    # Build the lookup table
    for routine_binary in routine_binary_streams:
        # Append the pointer to the lookup table
        final_binary.extend(ptr.to_bytes(2, byteorder='little', signed=False))
        ptr += len(routine_binary) # Advance the pointer by the length of the routine binary

    # Append the routine binaries to the final binary
    for routine_binary in routine_binary_streams:
        final_binary.extend(routine_binary)

    return final_binary

# Example usage
if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Compile routine scripts into a binary file for the science module EEPROM.")
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="routine_upload_packets.bin",
        help="Name of the output binary file (default: routine_upload_packets.bin)"
    )
    args = parser.parse_args()

    routine_folder_path = "routine_scripts"
    dictionary_path = "dictionary.csv"

    # Ensure paths are relative to the script's directory
    base_dir = os.path.dirname(__file__)
    routine_folder_path = os.path.join(base_dir, routine_folder_path)
    dictionary_path = os.path.join(base_dir, dictionary_path)

    # Read from memory map
    try:
        import science.science.config.mem_map_eval.just as just
        EEPROM_ROUTINE_START_ADDR = just.EEPROM_ROUTINE_LOOKUP_TABLE_SIZE_ADDR
    except:
        raise ValueError("EEPROM_ROUTINE_LOOKUP_TABLE_SIZE_ADDR not found in memory map.")

    # Substitute tokens and print the result
    eeprom_data = build_routine_array_binary(routine_folder_path, dictionary_path, EEPROM_ROUTINE_START_ADDR)
    packet_stream = SMFL.packetize_eeprom_write(eeprom_data, EEPROM_ROUTINE_START_ADDR, acknowledge=True)

    # Save the binary content to a file
    output_file_path = os.path.join(base_dir, args.output)
    with open(output_file_path, 'wb') as output_file:
        output_file.write(packet_stream)
    print(f"Compiled routine binary saved to {output_file_path}")

