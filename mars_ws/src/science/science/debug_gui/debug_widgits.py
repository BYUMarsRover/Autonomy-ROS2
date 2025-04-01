import struct
from PyQt5 import QtWidgets, uic, QtGui
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
from ament_index_python.packages import get_package_share_directory
from rover_msgs.msg import ScienceSerialTxPacket
import os
import sys

class DebugWindowWidget(QtWidgets.QWidget):
    def __init__(self, node):
        super(DebugWindowWidget, self).__init__()

        self.node = node

        # Find the UI file for the skeleton
        skeleton_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'skeleton.ui'
        )

        # Load the skeleton into this object
        uic.loadUi(skeleton_path, self)

        # Get the layout object
        self.layout_action = self.findChild(QtWidgets.QScrollArea, "scrollArea_command").widget().layout()

        # Add action widgets to the first scroll area
        self.action_widgets = []
        for func_definition in SMFL.filter({'command_type': 'action'}):
            widget = ActionWidget(node, self, func_definition)
            widget.setObjectName(f"commandWidget_{len(self.action_widgets)}")
            self.action_widgets.append(widget)
            self.layout_action.addWidget(widget)

        # Get the layout object
        self.layout_query = self.findChild(QtWidgets.QScrollArea, "scrollArea_query").widget().layout()

        # Add query widgets to the second scroll area
        self.query_widgets = []
        for func_definition in SMFL.filter({'command_type': 'query'}):
            widget = ActionWidget(node, self, func_definition)
            widget.setObjectName(f"queryWidget_{len(self.query_widgets)}")
            self.query_widgets.append(widget)
            self.layout_query.addWidget(widget)

    def packet_display(self, msg: ScienceSerialTxPacket):
        last_published_command_label = self.findChild(QtWidgets.QLabel, "lastSentLabel")
        last_published_command_label.setText(f"Last Packet Sent: {hex(msg.command_word)} {[ hex(x) for x in msg.operands ]}")

    def packet_modify_flags(self, command_word):
        command_word |= 0b01000000 if self.get_override_bit() else 0b00000000  # Set the Override Bit
        command_word |= 0b00100000 if self.get_ack_bit() else 0b00000000  # Set the Ack Bit
        return command_word
    
    def get_override_bit(self):
        override_box = self.findChild(QtWidgets.QCheckBox, "ovrBit")
        return override_box.isChecked()
    
    def get_ack_bit(self):
        ack_box = self.findChild(QtWidgets.QCheckBox, "ackBit")
        return ack_box.isChecked()
    
    def get_flags(self):
        return [self.get_override_bit(), self.get_ack_bit()]


class ActionWidget(QtWidgets.QWidget):
    def __init__(self, node, window, func):
        super(ActionWidget, self, ).__init__()

        self.node = node
        self.window = window

        # Find the UI file for the skeleton
        action_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'action_widget.ui'
        )

        self.func = func

        # Load the action widget into this object
        uic.loadUi(action_widget_path, self)

        # Get the layout object
        self.layout = self.findChild(QtWidgets.QGroupBox, "groupBox").layout()

        # Add operand widgets to the layout
        self.operand_widgets = []
        for i in [1, 2, 3]:
            # Get the next operand in this function definition
            name = func[f'operand_name_{i}']
            if name == '':
                break
            data_type = func[f'operand_type_{i}']

            cnt_str = func[f'operand_cnt_{i}']
            if (cnt_str == 'variable'):
                # Create the widget and add it to the layout
                widget = ArrayWidget(name, data_type, (1, sys.maxsize))
                widget.setObjectName(f"operandWidget_{len(self.operand_widgets)}")
                self.operand_widgets.append(widget)
                self.layout.insertWidget((len(self.operand_widgets) - 1) + 1, widget)
            else:
                # Create the widget and add it to the layout
                widget = OperandWidget(name, data_type)
                widget.setObjectName(f"operandWidget_{len(self.operand_widgets)}")
                self.operand_widgets.append(widget)
                self.layout.insertWidget((len(self.operand_widgets) - 1) + 1, widget)

        # Modify the Title
        groupBox = self.findChild(QtWidgets.QGroupBox, "groupBox")
        groupBox.setTitle(f"{func['function_name']} - {hex(SMFL.get_command_word(func))}")

        # Modify the Description
        descLabel = self.findChild(QtWidgets.QLabel, "descriptionLabel")
        descLabel.setText(f"{func['docstring']}")

        # Link the button to the function
        button = self.findChild(QtWidgets.QPushButton, "pushButton")
        button.clicked.connect(self.submit)

    def submit(self):
        # Convert Operands into byte array
        try:
            operand_bytes = []
            for widget in self.operand_widgets:
                b = widget.get_bytes()
                if type(b) is list:
                    operand_bytes.extend(b)
                else:
                    operand_bytes.append(b)
        except Exception as e:
            self.node.get_logger().error(f"Failed to read operands: {str(e)}")
            return
        
        try:
            tx_packet = SMFL.build_tx_packet(
                self.func,
                operand_bytes,
                self.window.get_flags()
            )
            # Publish packet
            self.node.packet_publish(tx_packet)
            self.node.update_last_published_packet(tx_packet)
        except Exception as e:
            self.node.get_logger().error(f"Failed to build tx_packet: {str(e)}")

class DataField():
    def __init__(self, data_type, lineEdit: QtWidgets.QLineEdit):
        self.data_type = data_type
        self.validator = DataField.assign_data_validation(self.data_type)
        self.lineEdit = lineEdit
        self.lineEdit.setValidator(self.validator)
        self.lineEdit.setPlaceholderText(f"{self.data_type}")

    def bytes(self):
        text = self.lineEdit.text()
        state, _, _ = self.validator.validate(text, 0)
        if state == QtGui.QValidator.Acceptable:
            return DataField.str2bytes(self.data_type, text)
        else:
            raise ValueError(f"Invalid input for data_type {self.data_type}: {text}")
    
    @staticmethod
    def assign_data_validation(data_type):
        if data_type == 'uint8_t':
            return QtGui.QIntValidator(0, 2**8 - 1)
        elif data_type == 'uint16_t':
            return QtGui.QIntValidator(0, 2**16 - 1)
        elif data_type == 'uint32_t':
            return QtGui.QIntValidator(0, 2**31 - 1)
        elif data_type == 'int8_t':
            return QtGui.QIntValidator(-(2**7), (2**7)-1)
        elif data_type == 'bool':
            return QtGui.QIntValidator(0, 1)
        elif data_type == 'float':
            return QtGui.QDoubleValidator()
        
    @staticmethod
    def str2bytes(data_type, value_str):
        try:
            if data_type in ['uint8_t', 'int8_t']:
                return [int(value_str) & 0xFF]
            elif data_type == 'uint16_t':
                value = int(value_str)
                return [value & 0xFF, (value >> 8) & 0xFF]
            elif data_type == 'uint32_t':
                value = int(value_str)
                return [value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]
            elif data_type == 'bool':
                return [1 if int(value_str) else 0]
            elif data_type == 'float':
                data = struct.pack('f', float(value_str))
                return [int(b) for b in data]
            else:
                raise Exception(f"Unknown data_type: {data_type}")
        except ValueError:
            raise Exception(f"Invalid input for data_type {data_type}: {value_str}")

class OperandWidget(QtWidgets.QWidget):
    def __init__(self, name, data_type):
        super(OperandWidget, self).__init__()

        # Save inputs
        self.name = name
        self.data_type = data_type

        # Find the UI file for the skeleton
        operand_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'operand.ui'
        )

        # Load the action widget into this object
        uic.loadUi(operand_widget_path, self)

        # Update the title
        self.groupBox.setTitle(name)

        # Connect the lineEdit
        self.data_field = DataField(self.data_type, self.lineEdit)

    def get_bytes(self):
        return self.data_field.bytes()

class ArrayEntryWidget(QtWidgets.QWidget):
    def __init__(self, index_num, data_type):
        super(ArrayEntryWidget, self).__init__()

        # Save inputs
        self.index_num = index_num
        self.data_type = data_type

        # Load widget UI file
        array_entry_widget_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'array_entry.ui'
        )
        uic.loadUi(array_entry_widget_path, self)

        # Update the index_num
        self.indexLabel.setText(str(index_num))

        # Connect the lineEdit
        self.data_field = DataField(self.data_type, self.lineEdit)

    def get_bytes(self):
        return self.data_field.bytes()
        
class ArrayWidget(QtWidgets.QWidget):
    def __init__(self, name, type, size_tuple):
        super(ArrayWidget, self).__init__()

        # Load inputs
        self.name = name
        self.type = type
        self.min_size = size_tuple[0]
        self.max_size = size_tuple[1]

        # Find the UI file for the skeleton
        operand_array_path = os.path.join(
            get_package_share_directory('science'),
            'debug_gui',
            'operand_array.ui'
        )

        # Load the action widget into this object
        uic.loadUi(operand_array_path, self)

        # Update the title
        self.groupBox.setTitle(name)

        # Get layout
        self.layout = self.groupBox.layout()

        # Add the buttons
        self.addButton.clicked.connect(self.add_array_entry)
        self.removeButton.clicked.connect(self.remove_array_entry)

        # Add the array entry widgets
        self.array_entry_widgets = []
        for i in range(self.min_size):
            self.add_array_entry()

    def array_size(self):
        return len(self.array_entry_widgets)

    def add_array_entry(self):
        widget = ArrayEntryWidget(self.array_size(), self.type)
        widget.setObjectName(f"arrayEntryWidget_{self.array_size()}")
        self.array_entry_widgets.append(widget)
        self.layout.insertWidget(self.array_size() - 1, widget)
        self.refresh_buttons()

    def remove_array_entry(self):
        widget = self.array_entry_widgets[self.array_size() - 1]
        self.layout.removeWidget(widget)
        self.array_entry_widgets.remove(widget)
        self.refresh_buttons()

    def refresh_buttons(self):
        self.addButton.setEnabled(self.array_size() < self.max_size)
        self.removeButton.setEnabled(self.array_size() > self.min_size)

    def get_bytes(self):
        data = []
        for widget in self.array_entry_widgets:
            b = widget.get_bytes()
            if type(b) is list:
                data.extend(b)
            else:
                data.append(b)
        return data