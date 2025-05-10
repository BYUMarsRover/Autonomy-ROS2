#!/usr/bin/python3

from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt
import science.routine_gui.resources
from python_qt_binding.QtCore import QObject, Signal
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from science.function_mapping.function_map import ScienceModuleFunctionList as SMFL
from science.function_mapping.function_map import ACTUATOR_COUNT, ACTUATOR_INDEX_PROBE, ACTUATOR_INDEX_AUGER, ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, ACTUATOR_INDEX_SECONDARY_CACHE, ACTUATOR_INDEX_SECONDARY_CACHE_DOOR, ACTUATOR_INDEX_DRILL
from std_msgs.msg import UInt8, Bool, Empty
from rover_msgs.msg import ScienceActuatorState, ScienceActuatorControl

import os
import sys

FULL_STEAM_FORWARD = 127
FULL_STEAM_BACKWARD = -128
FULL_STEAM_STOP = 0

class Signals(QObject):
    science_rx_signal = Signal()
    science_tx_signal = Signal()
    # sensor_save_signal = Signal(ScienceSaveSensor)
    # FAD_save_signal = Signal(ScienceSaveFAD)
    # notes_save_signal = Signal(ScienceSaveNotes)
    # fad_intensity_signal = Signal(ScienceFADIntensity)

class science_routine_GUI(Node):

    def __init__(self):
        
        super().__init__('science_routine_GUI')
        self.qt = QtWidgets.QWidget()

        ui_file_path = os.path.join(
            get_package_share_directory('science'),
            'routine_gui',
            'routine_gui.ui'
            )

        # Constants for actuator indices
        self.probe_arm_group_range = (10, 200)
        self.drill_arm_group_range = (10, 200)
        self.primary_door_range = (0, 30)
        self.secondary_frame_range = (100, 20)

        # Polling rates
        self.passive_poll_rate_ms = 5000
        self.active_poll_rate_ms = 100

        # List of polled actuators
        self.polled_actuators = [ACTUATOR_INDEX_PROBE, ACTUATOR_INDEX_AUGER, ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, ACTUATOR_INDEX_SECONDARY_CACHE]

        uic.loadUi(ui_file_path, self.qt)
        self.init_publishers()
        self.init_subscriptions()
        self.task_launcher_init()
        self.init_timer_tasks()
        self.qt.show() # Show the GUI

    def task_launcher_init(self):
        self.signals = Signals()

        # self.qt.packet_browser.itemSelectionChanged.connect(self.select_response_packet)
        # self.qt.pushButton_clear_packets.clicked.connect(self.clear_packets)  # Clear the packet browse

        # Advance Buttons
        self.qt.probe_forward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PROBE, FULL_STEAM_FORWARD))
        self.qt.probe_forward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PROBE, FULL_STEAM_STOP))

        self.qt.auger_forward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_AUGER, FULL_STEAM_FORWARD))
        self.qt.auger_forward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_AUGER, FULL_STEAM_STOP))

        self.qt.primary_forward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, FULL_STEAM_FORWARD))
        self.qt.primary_forward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, FULL_STEAM_STOP))

        self.qt.secondary_forward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_SECONDARY_CACHE, FULL_STEAM_FORWARD))
        self.qt.secondary_forward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_SECONDARY_CACHE, FULL_STEAM_STOP))

        self.qt.drill_forward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_DRILL, FULL_STEAM_FORWARD))
        self.qt.drill_forward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_DRILL, FULL_STEAM_STOP))

        # Reverse Buttons
        self.qt.probe_backward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PROBE, FULL_STEAM_BACKWARD))
        self.qt.probe_backward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PROBE, FULL_STEAM_STOP))

        self.qt.auger_backward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_AUGER, FULL_STEAM_BACKWARD))
        self.qt.auger_backward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_AUGER, FULL_STEAM_STOP))

        self.qt.primary_backward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, FULL_STEAM_BACKWARD))
        self.qt.primary_backward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_PRIMARY_CACHE_DOOR, FULL_STEAM_STOP))

        self.qt.secondary_backward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_SECONDARY_CACHE, FULL_STEAM_BACKWARD))
        self.qt.secondary_backward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_SECONDARY_CACHE, FULL_STEAM_STOP))

        self.qt.drill_backward.pressed.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_DRILL, FULL_STEAM_BACKWARD))
        self.qt.drill_backward.released.connect(lambda: self.update_control_actuator(ACTUATOR_INDEX_DRILL, FULL_STEAM_STOP))

        # Connect passive_rate_lineEdit to update passive_poll_rate_ms
        self.qt.passive_rate_lineEdit.editingFinished.connect(
            lambda: self.update_passive_poll_rate(self.qt.passive_rate_lineEdit.text())
        )
        self.qt.passive_rate_lineEdit.setText(str(self.passive_poll_rate_ms))  # Set initial value
        # Connect active_rate_lineEdit to update active_poll_rate_ms
        self.qt.active_rate_lineEdit.editingFinished.connect(
            lambda: self.update_active_poll_rate(self.qt.active_rate_lineEdit.text())
        )
        self.qt.active_rate_lineEdit.setText(str(self.active_poll_rate_ms))  # Set initial value

        # Stop All Button
        self.qt.stop_all_button.clicked.connect(lambda: self.kill_switch_pub.publish(Empty()))

    def update_control_actuator(self, actuator_index, control):
        # Update the actuator control based on the index and control value
        if actuator_index == ACTUATOR_INDEX_PROBE:
            self.control_probe_pub.publish(ScienceActuatorControl(control=control))
        elif actuator_index == ACTUATOR_INDEX_AUGER:
            self.control_auger_pub.publish(ScienceActuatorControl(control=control))
        elif actuator_index == ACTUATOR_INDEX_PRIMARY_CACHE_DOOR:
            self.control_primary_pub.publish(ScienceActuatorControl(control=control))
        elif actuator_index == ACTUATOR_INDEX_SECONDARY_CACHE:
            self.control_secondary_pub.publish(ScienceActuatorControl(control=control))
        elif actuator_index == ACTUATOR_INDEX_DRILL:
            self.control_drill_pub.publish(ScienceActuatorControl(control=control))
        self.update_polling_rate(actuator_index, control)

    def update_passive_poll_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.passive_poll_rate_ms = new_rate
                self.get_logger().info(f"Passive poll rate updated to {self.passive_poll_rate_ms} ms.")
            else:
                self.get_logger().warning("Passive poll rate must be a positive integer.")
        except ValueError:
            self.get_logger().warning("Invalid input for passive poll rate. Please enter a valid integer.")

    def update_active_poll_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.active_poll_rate_ms = new_rate
                self.get_logger().info(f"Active poll rate updated to {self.active_poll_rate_ms} ms.")
            else:
                self.get_logger().warning("Active poll rate must be a positive integer.")
        except ValueError:
            self.get_logger().warning("Invalid input for active poll rate. Please enter a valid integer.")

    def init_publishers(self):
        # Initialize publishers here
        self.pub_get_actuator_state = self.create_publisher(UInt8, '/science_get_actuator_state', 10)
        # Actuator controls
        self.control_probe_pub = self.create_publisher(ScienceActuatorControl, "/science_serial_probe", 10)
        self.control_auger_pub = self.create_publisher(ScienceActuatorControl, "/science_serial_auger", 10)
        self.control_primary_pub = self.create_publisher(ScienceActuatorControl, "/science_serial_primary_cache_door", 10)
        self.control_secondary_pub = self.create_publisher(ScienceActuatorControl, "/science_serial_secondary_cache", 10)
        self.control_drill_pub = self.create_publisher(ScienceActuatorControl, "/science_serial_drill", 10)
        self.kill_switch_pub = self.create_publisher(Empty, "/science_emergency_stop", 10)

    def init_subscriptions(self):
        # Initialize subscriptions here
        self.sub_actuator_updates = self.create_subscription(ScienceActuatorState, '/science_actuator_state_update', self.update_actuator_state, 10)

    def init_timer_tasks(self):
        # Initialize any timers or periodic tasks here
        self.position_timer_periods = [0 for _ in range(ACTUATOR_COUNT)]
        self.position_timers = [None for _ in range(ACTUATOR_COUNT)]
        for index in self.polled_actuators:
            self.update_timer(index, self.passive_poll_rate_ms)

    def update_timer(self, actuator_index, polling_rate_ms):
        current_rate = self.position_timer_periods[actuator_index]
        if current_rate != polling_rate_ms:
            # If it is a differnt rate
            self.position_timer_periods[actuator_index] = polling_rate_ms
            if self.position_timers[actuator_index] is not None:
                # Destroy the existing timer if it exists
                self.position_timers[actuator_index].destroy()
            self.position_timers[actuator_index] = self.create_actuator_position_timer(actuator_index, polling_rate_ms / 1000)

    def create_actuator_position_timer(self, actuator_index, period_ms):
        # Create a timer for a specific actuator position
        timer = self.create_timer(period_ms, lambda: self.pub_get_actuator_state.publish(UInt8(data=actuator_index)))
        # self.get_logger().info(f"Created timer for actuator {actuator_index} with period {timer.timer_period_ns / 1e9} seconds")
        return timer
    
    def update_actuator_state(self, msg: ScienceActuatorState):
        # Update the GUI with the new actuator state
        # This function will be called whenever a new ScienceActuatorState message is received

        self.update_state_table(msg)
        self.update_graphics(msg)
        self.update_polling_rate(msg.index, msg.control)

        # print(f"Recieved actuator state update of index {msg.index} with position {msg.position}, control {msg.control}, reserved {msg.reserved}")
    
    def update_state_table(self, msg: ScienceActuatorState):
        # Update the table row corresponding to the actuator index
        table = self.qt.actuator_state_table
        if table.rowCount() <= msg.index:
            table.setRowCount(msg.index + 1)

        # Update the table cells for the actuator
        table.setItem(msg.index, 0, QtWidgets.QTableWidgetItem(f"{msg.position}"))
        table.setItem(msg.index, 1, QtWidgets.QTableWidgetItem(f"{msg.control}"))
        table.setItem(msg.index, 2, QtWidgets.QTableWidgetItem(f"{msg.reserved}"))

    def update_polling_rate(self, actuator_index, control):
        if control != 0:
            self.update_timer(actuator_index, self.active_poll_rate_ms)
        elif control == 0:
            self.update_timer(actuator_index, self.passive_poll_rate_ms)

    def update_graphics(self, msg: ScienceActuatorState):
        if msg.index == ACTUATOR_INDEX_PROBE:
            self.update_actuator_graphic(self.qt.probe_frame, "probe_frame", self.qt.probe_arm_group, self.probe_arm_group_range, msg.position, msg.reserved)
        elif msg.index == ACTUATOR_INDEX_AUGER:
            self.update_actuator_graphic(self.qt.drill_frame, "drill_frame", self.qt.drill_arm_group, self.drill_arm_group_range, msg.position, msg.reserved)
        elif msg.index == ACTUATOR_INDEX_PRIMARY_CACHE_DOOR:
            self.update_actuator_graphic(self.qt.primary_cache_frame, "primary_cache_frame", self.qt.primary_door, self.primary_door_range, msg.position, msg.reserved, vertical=False)
        elif msg.index == ACTUATOR_INDEX_SECONDARY_CACHE:
            self.update_actuator_graphic(self.qt.secondary_frame, "secondary_frame", self.qt.secondary_frame, self.secondary_frame_range, msg.position, msg.reserved, vertical=False)
        # TODO add drill animations

    def update_actuator_graphic(self, frame, frame_label, arm_group, arm_range, position, reserved, vertical=True):
        # Update the actuator graphic based on the position and reserved values
        if reserved:
            frame.setStyleSheet(f"#{frame_label} {{background-color: rgb(246, 97, 81);}}")
        else:
            frame.setStyleSheet("")

        if vertical:
            # Adjust the y position of the arm group based on the position variable
            arm_group.setGeometry(
                arm_group.x(),
                int(map_range(  # Adjust y position according to position
                    position,
                    0,
                    255,
                    arm_range[0],
                    arm_range[1]
                )),
                arm_group.width(),
                arm_group.height()
            )
        else:
            # Adjust the x position of the arm group based on the position variable
            arm_group.setGeometry(
                int(map_range(  # Adjust x position according to position
                    position,
                    0,
                    255,
                    arm_range[0],
                    arm_range[1]
                )),
                arm_group.y(),
                arm_group.width(),
                arm_group.height()
            )

def map_range(value, from_min, from_max, to_min, to_max):
    """
    Maps a value from one range to another.

    :param value: The value to map.
    :param from_min: The minimum of the original range.
    :param from_max: The maximum of the original range.
    :param to_min: The minimum of the target range.
    :param to_max: The maximum of the target range.
    :return: The mapped value in the target range.
    """
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt application
    app = QtWidgets.QApplication(sys.argv)
    gui = science_routine_GUI()  # Initialize your GUI

    # Start the ROS 2 spin loop in a separate thread or via a timer
    def spin_ros():
        rclpy.spin_once(gui, timeout_sec=0.1)  # This will process ROS callbacks

    # Set up a timer to periodically call spin_ros inside the Qt event loop
    timer = gui.qt.startTimer(100)  # Spin ROS every 100ms
    
    # Override the timer event to call spin_ros
    def timer_event(event):
        spin_ros()  # Call spin_ros to process any incoming ROS messages

    gui.qt.timerEvent = timer_event  # Assign the timer event handler

    # Start the Qt event loop
    app.exec_()

    rclpy.shutdown()


if __name__ == "__main__":
    main()