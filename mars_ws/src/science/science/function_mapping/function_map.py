import os
from csv import DictReader
from ament_index_python.packages import get_package_share_directory
from rover_msgs.msg import ScienceSerialTxPacket, ScienceSerialRxPacket
import struct

ACTUATOR_COUNT = 6
ACTUATOR_INDEX_PROBE = 0
ACTUATOR_INDEX_AUGER = 1
ACTUATOR_INDEX_PRIMARY_CACHE_DOOR = 2
ACTUATOR_INDEX_SECONDARY_CACHE = 3
ACTUATOR_INDEX_SECONDARY_CACHE_DOOR = 4
ACTUATOR_INDEX_DRILL = 5

class ScienceModuleFunctionList:

    max_operands = 3

    path = os.path.join(
            get_package_share_directory('science'),
            'function_mapping',
            'science_module_function_map.csv'
        )
    with open(path, 'r') as f:
            dict_reader = DictReader(f)
            functions = list(dict_reader)

    @staticmethod
    def get_all():
        return ScienceModuleFunctionList.functions
    
    @staticmethod
    def filter(args):
        """Filter the function list based on the provided arguments."""
        filtered_list = []
        for func in ScienceModuleFunctionList.functions:
            skip = False
            for k, v in args.items():
                if not str(v) == func[k]:
                    skip = True
                    break
            if not skip:
                filtered_list.append(func)
        return filtered_list
    
    @staticmethod
    def build_tx_packet(func, operand_blob=[], flags=[False, False]):
        """
        Build a ScienceSerialTxPacket from a function entry.
        """
        # Setup the command word based on the function entry
        command_word = int(func['function_addr'])
        if func['command_type'] == 'action':
            command_word |= 0b10000000  # Set the command bit for command type functions
        for i in range(len(flags)):
            if flags[i]:
                command_word |= 0b00000001 << (6 - i)  # Set the override, and acknowledge bits

        # Verify the operand blob
        ScienceModuleFunctionList.verify_operands(func, operand_blob)

        # Build the packet and add the operands
        tx_packet = ScienceSerialTxPacket()
        tx_packet.command_word = ScienceModuleFunctionList.get_command_word(func, flags)
        tx_packet.operands = operand_blob
        return tx_packet
            
    
    @staticmethod
    def verify_operands(func, operand_blob):
        tracer = 0

        for i in range(1, ScienceModuleFunctionList.max_operands + 1):
            # Get the next operand in this function definition
            data_type = func[f'operand_type_{i}']
            if data_type == 'void':
                break

            # Get it's expected size
            size = ScienceModuleFunctionList.length_of_datatype(data_type)
            if size is None:
                raise Exception(f"Unrecognized data type encounterd while parsing operands: '{data_type}'")
            
            # Check if this is a variable length
            operand_count = func[f'operand_cnt_{i}'] 
            if operand_count == 'variable':

                # Ensure this is the last operand
                if i < ScienceModuleFunctionList.max_operands:
                    for j in range(i+1, ScienceModuleFunctionList.max_operands + 1):
                        if func[f'operand_type_{j}'] != 'void':
                            raise Exception("Encountered a operand with variable count which is not the final operand")
                
                # Iterate looking to land on the end of the array
                while True:
                    # Ran past the array while looking for inputs
                    if (tracer + size > len(operand_blob)):
                        raise Exception(f"Operand data length does not cleanly divide into the variable datatype {data_type}")
                    
                    # Grab the operand and attempt to interpret it
                    blob = operand_blob[tracer : tracer + size]
                    is_valid = ScienceModuleFunctionList.__verify_data(blob, data_type)
                    if not is_valid:
                        raise Exception(f"Unable to interpret provided {blob} as type {data_type}")
                    
                    # Advance to next item
                    tracer += size

                    # If cleanly divides, break
                    if tracer == len(operand_blob):
                        break

            # Iterate cnt times   
            else:
                for j in range(int(func[f'operand_cnt_{i}'])):
                    # Ran past the array while looking for inputs
                    if (tracer + size > len(operand_blob)):
                        raise Exception(f"Not enough data was provided for all expected operands. Expected at least {tracer + size} bytes, but got {len(operand_blob)} bytes. Operand index: {i}, Operand type: {data_type}, Function: {func['function_name']}")
                    
                    # Grab the operand and attempt to interpret it
                    blob = operand_blob[tracer : tracer + size]
                    is_valid = ScienceModuleFunctionList.__verify_data(blob, data_type)
                    if not is_valid:
                        raise Exception(f"Unable to interpret provided {blob} as type {data_type}")
                    
                    # Advance to next item
                    tracer += size

        # Catch the case where extra data was provided
        if tracer != len(operand_blob):
            raise Exception(f"More data was provided then expected for func {func['function_name']} len: {len(operand_blob)}")
        
    @staticmethod
    def __blob_data(number, datatype):
        try:
            if datatype in ['uint8_t', 'int8_t']:
                return [number & 0xFF]
            elif datatype == 'uint16_t':
                value = number
                return [value & 0xFF, (value >> 8) & 0xFF]
            elif datatype == 'uint32_t':
                value = number
                return [value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]
            elif datatype == 'bool':
                return [1 if number else 0]
            elif datatype == 'float':
                data = struct.pack('f', number)
                return [int(b) for b in data]
            else:
                raise Exception(f"Unknown data_type: {datatype}")
        except ValueError:
            raise Exception(f"Invalid input for data_type {datatype}: {number}")
        
    @staticmethod
    def __verify_data(data, datatype):
        try:
            if datatype == 'float':
                struct.unpack('<f', struct.pack('B' * len(data), *data))
            return True
        except:
            return False
    
    @staticmethod
    def length_of_datatype(datatype):
        match datatype:
            case 'uint8_t':
                result = 1
            case 'int8_t':
                result = 1
            case 'bool':
                result = 1
            case 'uint16_t':
                result = 2
            case 'uint32_t':
                result = 4
            case 'float':
                result = 4
            case 'void':
                result = 0
            case _:
                result = None
        return result
    
    @staticmethod
    def get_command_word(func, flags=[False, False]):
        """
        Build a ScienceSerialTxPacket from a function entry.
        """
        # Setup the command word based on the function entry
        command_word = int(func['function_addr'])
        if func['command_type'] == 'action':
            command_word |= 0b10000000  # Set the command bit for command type functions
        for i in range(len(flags)):
            if flags[i]:
                command_word |= 0b00000001 << (6 - i)  # Set the override, and acknowledge bits

        return command_word
    
    @staticmethod
    def get_function_by_command_word(command_word):
        results = ScienceModuleFunctionList.filter(
            {
                'command_type': 'action' if (command_word & 0b10000000) > 0 else 'query',
                'function_addr': command_word & 0b00011111
            }
        )
        if len(results) == 0:
            return None
        else:
            return results[0]
        
    @staticmethod
    def get_function_by_function_name(function_name):
        results = ScienceModuleFunctionList.filter(
            {
                'function_name': function_name
            }
        )
        if len(results) == 0:
            return None
        else:
            return results[0]
        
    @staticmethod
    def get_return_data(func_def, msg: ScienceSerialRxPacket):
        output = msg.message
        data_type = func_def['return_type']
        cnt = int(func_def['return_cnt'])
        size = ScienceModuleFunctionList.length_of_datatype(data_type)
        conversion_func = ScienceModuleFunctionList.conversion_function(data_type)

        out = []
        for i in range(cnt):
            out.append(conversion_func(output, i*size))
        return out[0] if cnt == 1 else out

    @staticmethod
    def conversion_function(datatype):
        match datatype:
            case 'uint8_t':
                result = lambda data, index=0: struct.unpack('<B', bytes(data[index:index+1]))[0]
            case 'int8_t':
                result = lambda data, index=0: struct.unpack('<b', bytes(data[index:index+1]))[0]
            case 'uint16_t':
                result = lambda data, index=0: struct.unpack('<H', bytes(data[index:index+2]))[0]
            case 'uint32_t':
                result = lambda data, index=0: struct.unpack('<I', bytes(data[index:index+4]))[0]
            case 'float':
                result = lambda data, index=0: struct.unpack('<f', bytes(data[index:index+4]))[0]
            case 'bool':
                result = lambda data, index=0: bool(struct.unpack('<B', bytes(data[index:index+1]))[0])
            case 'void':
                result = lambda data, index=0: None
            case _:
                raise Exception(f"Unknown data_type: {datatype}")
        return result
        
    # Mapping Functions
    
    @staticmethod
    def get_tx_get_analog_sensor_raw(sensor_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("get_analog_sensor_raw"),
            operand_blob = [sensor_index]
        )
    
    @staticmethod
    def get_tx_get_analog_sensor_calibrated(sensor_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("get_analog_sensor_calibrated"),
            operand_blob = [sensor_index]
        )
    
    @staticmethod
    def get_tx_get_actuator_position(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("get_actuator_position"),
            operand_blob = [actuator_index]
        )
    
    @staticmethod
    def get_tx_get_actuator_control(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("get_actuator_control"),
            operand_blob = [actuator_index]
        )
    
    @staticmethod
    def get_tx_query_is_actuator_reserved(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("query_is_actuator_reserved"),
            operand_blob = [actuator_index]
        )
    
    @staticmethod
    def get_tx_update_actuator_control(actuator_index, control):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("update_actuator_control"),
            operand_blob = [actuator_index, control & 0xFF] # Make Control be interpreted as uint8_t
        )
    
    @staticmethod
    def get_tx_sample_spectrograph(sample_cnt, sample_interval_ms, bulb_on):
        sample_interval_blob = ScienceModuleFunctionList.__blob_data(sample_interval_ms, 'uint32_t')
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("sample_spectrograph"),
            operand_blob = [sample_cnt] + sample_interval_blob + [bulb_on]
        )
    
    @staticmethod
    def get_tx_return_spectrograph_data():
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("return_spectrograph_data")
        )
    
    @staticmethod
    def get_tx_sample_ltr():
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("sample_ltr")
        )
    
    @staticmethod
    def get_tx_return_ltr_data():
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("return_ltr_data")
        )
    
    @staticmethod
    def get_tx_abort_routine():
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("abort_routine")
        )
    
    @staticmethod
    def get_tx_clear_speed_controller(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("clear_speed_controller"),
            operand_blob = [actuator_index]
        )
    
    @staticmethod
    def get_tx_clear_positional_controller(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("clear_positional_controller"),
            operand_blob = [actuator_index]
        )
    
    @staticmethod
    def get_tx_free_actuator(actuator_index):
        return ScienceModuleFunctionList.build_tx_packet(
            ScienceModuleFunctionList.get_function_by_function_name("free_actuator"),
            operand_blob = [actuator_index]
        )
        