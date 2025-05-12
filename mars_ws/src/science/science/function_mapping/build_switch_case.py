import argparse
import datetime
import csv
import os

def csv_to_list(file_path):
    """
    Reads a CSV file and converts it into a list of dictionaries.

    Args:
        file_path (str): The path to the CSV file.

    Returns:
        list[dict]: A list of dictionaries where each dictionary represents a row in the CSV file.
    """
    data = []
    with open(file_path, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            data.append(row)
    return data

def auto_generated_header():
    """
    Generates a standard auto-generated file header comment.

    Returns:
        str: A string containing the header comment.
    """
    header = (
        "/*\n"
        " * AUTO-GENERATED FILE\n"
        " *\n"
        f" * This file was automatically generated on {datetime.date.today().strftime('%d %B %Y')}.\n"
        " * It directs to the appropriate function call using the command word and function address.\n"
        " *\n"
        " * DO NOT MODIFY THIS FILE DIRECTLY.\n"
        " * To update the contents of this file, please edit the function mapping CSV file\n"
        " * ('science_module_function_map.csv') and run the script 'build_switch_case.py'.\n"
        " *\n"
        " * Any manual changes to this file will be overwritten the next time the script is executed.\n"
        " */\n"
    )
    return header

def build_switch_board(data, fileout, headers):
    """
    Defines the `executeQuery` and `executeAction` functions to route to each science module function.

    Args:
        data (list[dict]): A list of dictionaries containing function definitions.
                           Each dictionary should have the keys:
                           - "command_type": The type of command ("query" or "action").
                           - "function_addr": The address of the function.
                           - "function_name": The name of the function.
        fileout (str): The path to the output file where the switch board will be written.
        headers (list[str]): The file paths to included header files

    Writes:
        A C++ file containing the `executeQuery` and `executeAction` functions with switch cases
        for routing commands to the appropriate functions.
    """
    # Parse the Arduino switch case strings
    query_case_strings = []
    action_case_strings = []
    for row in data:
        if row["command_type"] == "query":
            query_case_strings.append(f'case {row["function_addr"]}: {row["function_name"]}(override); break;')
        else:
            action_case_strings.append(f'case {row["function_addr"]}: {row["function_name"]}(override); break;')

    # Build the execute query function
    execute_query_function_str = "void executeQuery(uint8_t function_addr, uint8_t override) {\n    switch (function_addr) {\n"
    for case in query_case_strings:
        execute_query_function_str += f"        {case}\n"
    execute_query_function_str += "        default:\n            // No query configured\n            error::badFunctionAddress(function_addr);\n            break;\n    }\n}\n"

    # Build the execute action function
    execute_action_function_str = "void executeAction(uint8_t function_addr, uint8_t override) {\n    switch (function_addr) {\n"
    for case in action_case_strings:
        execute_action_function_str += f"        {case}\n"
    execute_action_function_str += "        default:\n            // No command configured\n            error::badFunctionAddress(function_addr);\n            break;\n    }\n}\n"

    # Write the output to the file
    with open(fileout, mode='w') as file:
        file.write("#include <stdint.h>\n")
        file.write(auto_generated_header())
        file.write("\n")
        file.write("#include <stdint.h>\n")
        file.write('#include "../error/error.h"\n')
        for h in headers:
            file.write(f'#include "{h}"\n')
        file.write("\n")
        file.write(execute_query_function_str)
        file.write("\n")
        file.write(execute_action_function_str)
        file.write("\n")

if __name__ == "__main__":
    """
    Main entry point for the script. Parses command-line arguments and generates the switch board file.

    Command-Line Arguments:
        csv (str): The input file path for the function definitions (CSV file).
        fileout (str): The output file path for the generated switch board.

    Example Usage:
        python build_switch_case.py science_module_function_map.csv output_file.cpp
    """
    parser = argparse.ArgumentParser(description="Build the Arduino switch board file.")
    parser.add_argument("csv", help="The input file path for the function definitions.")
    parser.add_argument("fileout", help="The output file path for the generated switch board.")
    parser.add_argument(
        "--headers", 
        nargs="*", 
        default=[], 
        help="Header files to include in the generated file."
    )
    args = parser.parse_args()

    # Get the CSV data
    data = csv_to_list(args.csv)
    build_switch_board(data, args.fileout, args.headers)