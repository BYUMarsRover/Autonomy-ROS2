import argparse
import datetime
import csv

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
        " *\n"
        " * DO NOT MODIFY THIS FILE DIRECTLY.\n"
        " * To update the contents of this file, please edit the function mapping CSV file\n"
        " * ('science_module_function_map.csv') and run the script 'build_function_headers.py'.\n"
        " *\n"
        " * Any manual changes to this file will be overwritten the next time the script is executed.\n"
        " */\n"
    )
    return header

def build_function_headers(data, fileout):
    """
    Builds a header file containing function declarations for each science module function.

    Args:
        data (list[dict]): A list of dictionaries containing function definitions.
                           Each dictionary should have the keys:
                           - "function_name": The name of the function.
                           - "docstring": A brief description of the function.
        fileout (str): The path to the output header file.

    Writes:
        A header file containing function declarations and an auto-generated header comment.
    """
    # Parse the Arduino function declarations
    function_headers = []
    for row in data:
        function_headers.append(f'void {row["function_name"]}(uint8_t override); // {row["docstring"]}')

    # Build the header file content
    execute_action_function_str = ""
    for header in function_headers:
        execute_action_function_str += (header + '\n')

    # Write the header file
    with open(fileout, mode='w') as file:
        file.write("#ifndef SM_FUNCTIONS_H\n")
        file.write("#define SM_FUNCTIONS_H\n\n")
        file.write(auto_generated_header())
        file.write("\n")
        file.write("#include <stdint.h>\n")
        file.write("\n")
        file.write(execute_action_function_str)
        file.write("\n#endif /* SM_FUNCTIONS_H */\n")

if __name__ == "__main__":
    """
    Main entry point for the script. Parses command-line arguments and generates the function header file.

    Command-Line Arguments:
        csv (str): The input file path for the function definitions (CSV file).
        fileout (str): The output file path for the generated function header file.

    Example Usage:
        python build_function_headers.py science_module_function_map.csv output_file.h
    """
    parser = argparse.ArgumentParser(description="Build the Arduino function header file.")
    parser.add_argument("csv", help="The input file path for the function definitions.")
    parser.add_argument("fileout", help="The output file path for the generated function header.")
    args = parser.parse_args()

    # Get the CSV data
    data = csv_to_list(args.csv)
    build_function_headers(data, args.fileout)