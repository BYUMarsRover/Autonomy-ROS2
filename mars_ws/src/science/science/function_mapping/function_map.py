import os
from csv import DictReader
import struct

try:
    from ament_index_python.packages import get_package_share_directory
    from rover_msgs.msg import ScienceSerialTxPacket, ScienceSerialRxPacket
except ImportError:
    # Fallback for non-ROS environments
    def get_package_share_directory(package_name):
        # Define a fallback directory for the package
        base_dir = os.path.dirname(__file__)
        return os.path.abspath(os.path.join(base_dir, "..", "..", package_name))

ACTUATOR_COUNT = 6
ACTUATOR_INDEX_PROBE = 0
ACTUATOR_INDEX_AUGER = 1
ACTUATOR_INDEX_PRIMARY_CACHE_DOOR = 2
ACTUATOR_INDEX_SECONDARY_CACHE_DOOR = 3
ACTUATOR_INDEX_SECONDARY_CACHE = 4
ACTUATOR_INDEX_DRILL = 5

ENTIRE_MESSAGE = -1

class ScienceModuleFunctionList:
    max_operands = 3

    # Use the fallback or ROS-based path resolution
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
    def verify_operands(func, operand_blob):
        tracer = 0

        print('')

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
    def blob_data(number, datatype):
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
        if datatype == 'uint8_t':
            return 1
        elif datatype == 'int8_t':
            return 1
        elif datatype == 'bool':
            return 1
        elif datatype == 'uint16_t':
            return 2
        elif datatype == 'uint32_t':
            return 4
        elif datatype == 'float':
            return 4
        elif datatype == 'char_array':
            return ENTIRE_MESSAGE
        elif datatype == 'void':
            return 0
        else:
            return None
    
    @staticmethod
    def get_command_word(func, flags=[False, False]):
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
    def conversion_function(datatype):
        if datatype == 'uint8_t':
            return lambda data, index=0: struct.unpack('<B', bytes(data[index:index+1]))[0]
        elif datatype == 'int8_t':
            return lambda data, index=0: struct.unpack('<b', bytes(data[index:index+1]))[0]
        elif datatype == 'uint16_t':
            return lambda data, index=0: struct.unpack('<H', bytes(data[index:index+2]))[0]
        elif datatype == 'uint32_t':
            return lambda data, index=0: struct.unpack('<I', bytes(data[index:index+4]))[0]
        elif datatype == 'float':
            return lambda data, index=0: struct.unpack('<f', bytes(data[index:index+4]))[0]
        elif datatype == 'bool':
            return lambda data, index=0: bool(struct.unpack('<B', bytes(data[index:index+1]))[0])
        elif datatype == 'char_array':
            return lambda data, index=0: ''.join(chr(b) for b in data[index:])
        elif datatype == 'void':
            return lambda data, index=0: None
        else:
            raise Exception(f"Unknown data_type: {datatype}")
    
    @staticmethod
    def __split_into_packets(data, max_packet_size):
        packets = []
        for i in range(0, len(data), max_packet_size):
            packets.append(data[i:i + max_packet_size])
        return packets
    
    @staticmethod
    def packetize_eeprom_write(final_binary, start_addr, acknowledge=False):
        '''Creats a final binary with science module packets'''

        HEADER = 0x53
        COMMAND = ScienceModuleFunctionList.get_command_word(ScienceModuleFunctionList.get_function_by_function_name('write_eeprom'), [False, acknowledge])
        FOOTER = 0x4D
        packet_stream = bytearray()
        payloads = ScienceModuleFunctionList.__split_into_packets(final_binary, 0xFF - 2)  # Split into maximum 255 bytes, minus two for eeprom location
        for payload in payloads:
            packet = bytearray()
            packet.append(HEADER)
            packet.append(COMMAND)
            packet.append(len(payload) + 2)
            packet.extend(start_addr.to_bytes(2, byteorder='little', signed=False))
            start_addr += len(payload)  # Increment the start address for the next packet
            packet.extend(payload)
            packet.append(FOOTER)
            packet_stream.extend(packet)
        return packet_stream

try:
    class ScienceModuleFunctionListBuilder:
        
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
        def get_return_data(func_def, msg: ScienceSerialRxPacket):
            output = msg.message
            data_type = func_def['return_type']
            conversion_func = ScienceModuleFunctionList.conversion_function(data_type)
            size = ScienceModuleFunctionList.length_of_datatype(data_type)
            if size is ENTIRE_MESSAGE:
                size = len(msg.message) # Use entire message in function
            if func_def['return_cnt'] == 'variable':
                # Determine the size at runtime
                cnt = int(len(msg.message) / size)
            else:
                # Pre determined size
                cnt = int(func_def['return_cnt'])
            out = []
            for i in range(cnt):
                out.append(conversion_func(output, i*size))
            return out[0] if cnt == 1 else out
        
        @staticmethod
        def get_tx_get_analog_sensor_raw(sensor_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("get_analog_sensor_raw"),
                operand_blob = [sensor_index]
            )
        
        @staticmethod
        def get_tx_get_analog_sensor_calibrated(sensor_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("get_analog_sensor_calibrated"),
                operand_blob = [sensor_index]
            )
        
        @staticmethod
        def get_tx_get_actuator_position(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("get_actuator_position"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_get_actuator_control(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("get_actuator_control"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_query_is_actuator_reserved(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("query_is_actuator_reserved"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_update_actuator_control(actuator_index, control, override=False):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("update_actuator_control"),
                operand_blob = [actuator_index, control & 0xFF], # Make Control be interpreted as uint8_t
                flags = [override, False]
            )
        
        @staticmethod
        def get_tx_sample_spectrograph(sample_cnt, sample_interval_ms, bulb_on):
            sample_interval_blob = ScienceModuleFunctionList.blob_data(sample_interval_ms, 'uint32_t')
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("sample_spectrograph"),
                operand_blob = [sample_cnt] + sample_interval_blob + [bulb_on]
            )
        
        @staticmethod
        def get_tx_return_spectrograph_data():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("return_spectrograph_data")
            )
        
        @staticmethod
        def get_tx_sample_ltr():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("sample_ltr")
            )
        
        @staticmethod
        def get_tx_return_ltr_data():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("return_ltr_data")
            )
        
        # Routine Methods
        
        @staticmethod
        def get_tx_run_routine(routine_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("run_routine"),
                operand_blob = [routine_index]
            )
        
        @staticmethod
        def get_tx_pause_routine():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("pause_routine")
            )
        
        @staticmethod
        def get_tx_resume_routine():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("resume_routine")
            )
        
        @staticmethod
        def get_tx_step_routine():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("step_routine")
            )
        
        @staticmethod
        def get_tx_abort_routine():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("abort_routine")
            )
        
        @staticmethod
        def get_tx_query_routine_controller():
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("query_routine_controller")
            )
        
        # Controllers
        
        @staticmethod
        def get_tx_clear_speed_controller(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("clear_speed_controller"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_clear_positional_controller(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("clear_positional_controller"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_free_actuator(actuator_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("free_actuator"),
                operand_blob = [actuator_index]
            )
        
        @staticmethod
        def get_tx_calibrate_uv_index(uv_index):
            return ScienceModuleFunctionListBuilder.build_tx_packet(
                ScienceModuleFunctionList.get_function_by_function_name("calibrate_uv_index"),
                operand_blob = ScienceModuleFunctionList.blob_data(uv_index, 'float')
            )
        
except NameError:
    # Fallback for non-ROS environments
    class ScienceModuleFunctionListBuilder:
        pass
