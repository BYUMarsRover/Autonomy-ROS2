import argparse
import csv
import datetime
import os
import re

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

def refresh_file_definitions(data, file, header_path):
    """
    Ensures that all functions defined in the provided CSV mapping file are present in the target file.
    Adds missing function definitions and maintains existing ones.

    Args:
        data (list[dict]): A list of dictionaries containing function definitions.
        file (str): The path to the file to update with function implementations.

    Writes:
        Updates the target file with function definitions, #include statements, and #define statements.
    """
    # Create the file if it doesn't exist
    if not os.path.exists(file):
        with open(file, mode='w') as new_file:
            new_file.write("// File Start")

    # Contains all function definitions with corresponding code and location in the file
    function_code_dict = {}

    # Identify existing functions in the file
    with open(file, mode='r') as file_obj:
        lines = file_obj.readlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if re.match(r"^void\s+\w+\s*\([^)]*\)\s*.*", line):
                function_name = line.split()[1].split('(')[0]
                code = ""
                brace_count = 1
                i += 1 # Move to next line
                while i < len(lines):
                    line = lines[i]
                    if '{' in line:
                        brace_count += line.count('{')
                    if '}' in line:
                        brace_count -= line.count('}')
                    if brace_count == 0:
                        break
                    else:
                        code += line
                    i += 1
                function_code_dict[function_name] = code
            else:
                i += 1

    # Identify missing functions and add to the dictionary
    for func_def in data:
        if func_def["function_name"] not in function_code_dict.keys():
            function_code_dict[func_def["function_name"]] = None

    # Identify existing #include statements
    include_set = []
    with open(file, mode='r') as file_obj:
        for line in file_obj:
            line = line.strip()
            if re.match(r'^#include\s+[<"].+[>"]', line):
                include_name = re.findall(r'[<"](.+?)[>"]', line)[0]
                include_set.append(include_name)
    for include in ["../error/error.h", "../packet_comm/packet_comm.h", header_path]:
        if include not in include_set:
            include_set.append(include)

    # Identify existing #define statements
    define_set = []
    with open(file, mode='r') as file_obj:
        for line in file_obj:
            line = line.strip()
            if line.startswith("#define"):
                define_set.append(line)
    
    # Write the updated content to the file
    with open(file, mode='w') as file_obj:
        file_obj.write(
            "/*\n"
            " * SCRIPT MAINTAINED FUNCTION DEFINITIONS\n"
            " *\n"
            f" * Generated on: {datetime.date.today().strftime('%d %B %Y')}\n"
            " *\n"
            " * This file is maintained by the 'build_function_implementations.py' script.\n"
            " * It ensures that all functions defined in the provided CSV mapping file\n"
            " * ('science_module_function_map.csv') are present in this file\n"
            " * and adds any missing function definitions.\n"
            " *\n"
            " * What you CAN modify:\n"
            " * - You may edit the function bodies to implement the desired functionality.\n"
            " * - You may add and remove #include statements and #define statements.\n"
            " *\n"
            " * What you CANNOT modify:\n"
            " * - Do not manually add or remove function definitions in this file.\n"
            " * - Do not modify the auto-generated include statements or function headers.\n"
            " *\n"
            " * Any manual changes to the auto-generated sections will be overwritten\n"
            " * the next time the script is executed.\n"
            " */\n\n"
        )

        # Reinsert the includes
        for include in include_set:
            file_obj.write(f'#include "{include}"\n')
        file_obj.write('\n')

        # Reinsert the defines
        if len(define_set) > 0:
            for define in define_set:
                file_obj.write(define)
                file_obj.write('\n')
            file_obj.write('\n')

        # Reinsert the functions
        for func_def in data:
            func_str = function_string(func_def, function_code_dict[func_def['function_name']])
            func_str = update_function_operand_error(func_def, func_str)
            file_obj.write(func_str)
            file_obj.write('\n')

def update_function_operand_error(func_def, code_str):
    """
    Updates the `verifyCommandOperandLength` call in the function code with the correct operand length.

    Args:
        func_def (dict): A dictionary containing the function definition.
        code_str (str): The existing function code as a string.

    Returns:
        str: The updated function code with the correct operand length.
    """
    if (length := get_total_operand_length(func_def)) is not None:
        code_str = re.sub(
            r"verifyCommandOperandLength\(\d+\)",
            f"verifyCommandOperandLength({length})",
            code_str
        )
    return code_str

def function_string(func_def, code=None):
    """
    Generates the string representation of a function definition.

    Args:
        func_def (dict): A dictionary containing the function definition.
        code (str, optional): The existing function code. Defaults to None.

    Returns:
        str: The string representation of the function definition.
    """
    str = ''

    # Handle doctsring comment
    comment = func_def["docstring"] if len(func_def["docstring"]) > 0 else "No Docstring Provided"
    line_start = 0
    for i in range(len(comment)):
        if (i - line_start > 80 and comment[i] == ' ') or (comment[i]=='\n') or (i == len(comment) - 1):
            str += f'// {comment[line_start:i+1]}\n'
            line_start = i+1

    # Handle operand documentation comments
    operand_array_index = 0
    for i in [1, 2, 3]:
        name = func_def[f'operand_name_{i}']
        if name == "":
            continue
        datatype = func_def[f'operand_type_{i}']
        size = length_of_datatype(datatype)
        cnt = func_def[f'operand_cnt_{i}']
        operand_doc_str = f'//    {i}: {name} ({datatype}) ['
        if size > 1:
            operand_doc_str += f'{operand_array_index}-{operand_array_index + size - 1}]\n'
        else:
            operand_doc_str += f'{operand_array_index}]\n'
        operand_array_index += size
        str += operand_doc_str

    str += f'void {func_def["function_name"]}(uint8_t override) {{\n'

    if code == None:
        # Write operand check command
        if (length := get_total_operand_length(func_def)) is not None:
            str += f'    if (!verifyCommandOperandLength({length})) return;\n'

        # Handle operands
        operand_array_index = 0
        for i in [1, 2, 3]:
            name = func_def[f'operand_name_{i}']
            if name == "":
                continue
            datatype = func_def[f'operand_type_{i}']
            size = length_of_datatype(datatype)
            operand_str = f'    {datatype} {name} = command_operand_buffer['
            if size > 1:
                operand_str += f'{operand_array_index}-{operand_array_index + size - 1}];\n'
            else:
                operand_str += f'{operand_array_index}];\n'
            operand_array_index += size
            str += operand_str

        # Default Error
        str += f'    // TODO: Implement {func_def["function_name"]}\n'
        str += f'    error::notImplemented();\n'
    else:
        str += code
        
    str += f'}}\n'
    return str

def get_total_operand_length(func):
    """
    Calculates the total operand length for a function based on its operand count and types.

    Args:
        func (dict): A dictionary containing the function definition.

    Returns:
        int: The total operand length in bytes, or None if the length cannot be determined.
    """
    cnt = 0
    for i in [1, 2, 3]:
        num = func[f'operand_cnt_{i}']
        datatype = func[f'operand_type_{i}']
        if length_of_datatype(datatype) is None:
            raise Exception(f"Datatype {datatype} is not recognized")
        try:
            cnt += int(num) * length_of_datatype(datatype)
        except:
            return None
    return cnt

def length_of_datatype(datatype):
    """
    Determines the size of a given datatype in bytes.

    Args:
        datatype (str): The name of the datatype.

    Returns:
        int: The size of the datatype in bytes, or None if the datatype is not recognized.
    """
    match datatype:
        case 'uint8_t':
            result = 1
        case 'int8_t':
            result = 1
        case 'uint16_t':
            result = 2
        case 'uint32_t':
            result = 4
        case 'float':
            result = 4
        case 'void':
            result = 0
        case 'bool':
            result = 1
        case 'calibration_point_t':
            result = 6
        case _:
            result = None
    return result

if __name__ == "__main__":
    """
    Main entry point for the script. Parses command-line arguments and generates or updates
    the Arduino function implementations file.

    Command-Line Arguments:
        csv (str): Path to the CSV file containing function definitions.
        file (str): Path to the file to receive the function implementations.

    Example Usage:
        python build_function_implementations.py science_module_function_map.csv output_file.cpp
    """
    parser = argparse.ArgumentParser(description="Generate and maintain Arduino function implementations.")
    parser.add_argument("csv", help="Path to the CSV file containing function definitions.")
    parser.add_argument("file", help="Path to the file to receive the function implementations.")
    parser.add_argument("header_path", help="Path to the header file with the function declarations.")
    args = parser.parse_args()

    # Get the CSV data
    data = csv_to_list(args.csv)
    refresh_file_definitions(data, args.file, args.header_path)
