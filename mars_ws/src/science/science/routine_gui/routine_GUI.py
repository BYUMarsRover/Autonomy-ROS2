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
from std_msgs.msg import UInt8, Bool, Empty, String
from rover_msgs.msg import ScienceActuatorState, ScienceActuatorControl, ScienceSerialTxPacket

import os
import sys

FULL_STEAM_FORWARD = 127
FULL_STEAM_BACKWARD = -127
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

        # Polling rates for actuators
        self.routine_status_query_rate_ms = 5000

        # Available Routines
        self.routines = ["reset_routine", "move_to_zero_routine", "test_routine", "first_cache_transfer"]

        uic.loadUi(ui_file_path, self.qt)
        self.init_publishers()
        self.init_subscriptions()
        self.init_timer_tasks()
        self.task_launcher_init()
        self.init_actuator_state_queries()
        self.qt.show() # Show the GUI

    def init_actuator_state_queries(self):

        # Initialize the polling manager for actuator states
        self.actuator_poll_manager = ActuatorStatePollManager(
            [
                ActuatorStatePoll(self, self.pub_get_actuator_state, ACTUATOR_INDEX_PROBE),
                ActuatorStatePoll(self, self.pub_get_actuator_state, ACTUATOR_INDEX_AUGER),
                ActuatorStatePoll(self, self.pub_get_actuator_state, ACTUATOR_INDEX_PRIMARY_CACHE_DOOR),
                ActuatorStatePoll(self, self.pub_get_actuator_state, ACTUATOR_INDEX_SECONDARY_CACHE),
                ActuatorStatePoll(self, self.pub_get_actuator_state, ACTUATOR_INDEX_DRILL)
            ],
            passive_rate_ms=1000,
            active_rate_ms=100
        )

        # Passive rate line Edit
        self.qt.passive_rate_lineEdit.editingFinished.connect(
            lambda: self.update_passive_poll_rate(self.qt.passive_rate_lineEdit.text())
        )
        self.qt.passive_rate_lineEdit.setText(str(self.actuator_poll_manager.passive_rate_ms))  # Set initial value

        # Active rate line Edit
        self.qt.active_rate_lineEdit.editingFinished.connect(
            lambda: self.update_active_poll_rate(self.qt.active_rate_lineEdit.text())
        )
        self.qt.active_rate_lineEdit.setText(str(self.actuator_poll_manager.active_rate_ms))  # Set initial value

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

        # Stop All Button
        self.qt.stop_all_button.clicked.connect(lambda: self.kill_switch_pub.publish(Empty()))

        # Routine Buttons
        self.qt.pause_routine_button.clicked.connect(
            lambda: self.pub_tx.publish(SMFL.get_tx_pause_routine())
        )
        self.qt.resume_routine_button.clicked.connect(
            lambda: self.pub_tx.publish(SMFL.get_tx_resume_routine())
        )
        self.qt.step_routine_button.clicked.connect(
            lambda: self.pub_tx.publish(SMFL.get_tx_step_routine())
        )
        self.qt.abort_routine_button.clicked.connect(
            lambda: self.pub_tx.publish(SMFL.get_tx_abort_routine())
        )

        # Begin Routine Buttons
        for index, routine in enumerate(self.routines):
            button = QtWidgets.QPushButton(routine)
            button.setObjectName(f"routine_{index}")
            button.clicked.connect(lambda _, i=index: self.pub_tx.publish(SMFL.get_tx_run_routine(i)))  # Capture index with default argument
            self.qt.routine_begin_buttons.layout().addWidget(button)

        # Query Routine Status Rate
        # Connect routine_status_query_rate_lineEdit to update routine_status_query_rate_ms
        self.qt.routine_status_query_rate_lineEdit.editingFinished.connect(
            lambda: self.update_routine_status_query_rate(self.qt.routine_status_query_rate_lineEdit.text())
        )
        self.qt.routine_status_query_rate_lineEdit.setText(str(self.routine_status_query_rate_ms))  # Set initial value

        # Manual Query
        self.qt.routine_status_refresh_button.clicked.connect(lambda: self.perform_routine_status_query())

        # Do one query to start
        self.perform_routine_status_query()

    def perform_routine_status_query(self):
        # Perform a routine status query
        self.pub_get_routine_state.publish(Empty())

    def update_control_actuator(self, actuator_index, control):
        # Update the actuator control using the tx_packet_builder in SMFL
        override_flag = self.qt.override_check.isChecked()
        tx_packet = SMFL.get_tx_update_actuator_control(actuator_index, control, override=override_flag)
        self.pub_tx.publish(tx_packet)
        self.actuator_poll_manager.update_attention(actuator_index, control)

    def update_passive_poll_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.actuator_state_passive_poll_rate_ms = new_rate
                self.get_logger().info(f"Passive poll rate updated to {self.actuator_state_passive_poll_rate_ms} ms.")
                self.actuator_poll_manager.update_passive_rate_ms(new_rate)
            else:
                self.get_logger().warning("Passive poll rate must be a positive integer.")
        except ValueError:
            self.get_logger().warning("Invalid input for passive poll rate. Please enter a valid integer.")

    def update_active_poll_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.actuator_active_poll_rate_ms = new_rate
                self.get_logger().info(f"Active poll rate updated to {self.actuator_active_poll_rate_ms} ms.")
                self.actuator_poll_manager.update_active_rate_ms(new_rate)
            else:
                self.get_logger().warning("Active poll rate must be a positive integer.")
        except ValueError:
            self.get_logger().warning("Invalid input for active poll rate. Please enter a valid integer.")

    def update_routine_status_query_rate(self, new_rate):
        try:
            new_rate = int(new_rate)
            if new_rate > 0:
                self.get_logger().info(f"Routine status query rate updated to {self.routine_status_query_rate_ms} ms.")
                self.update_routine_status_timer(new_rate)
            else:
                self.get_logger().warning("Routine status query rate must be a positive integer.")
        except ValueError:
            self.get_logger().warning("Invalid input for routine status query rate. Please enter a valid integer.")

    def init_publishers(self):
        # Initialize publishers here

        # Actuator state requests
        self.pub_get_actuator_state = self.create_publisher(UInt8, '/science_get_actuator_state', 10)

        # Routine State requests
        self.pub_get_routine_state = self.create_publisher(Empty, '/science_get_routine_status', 10)

        # Actuator controls
        self.kill_switch_pub = self.create_publisher(Empty, "/science_emergency_stop", 10)

        # Generic tx requests
        self.pub_tx = self.create_publisher(ScienceSerialTxPacket, '/science_serial_tx_request', 10)

    def init_subscriptions(self):
        # Initialize subscriptions here
        self.sub_actuator_updates = self.create_subscription(ScienceActuatorState, '/science_actuator_state_update', self.update_actuator_state, 10)

        # Subscribe to the /science_routine_status_update topic
        self.sub_routine_status = self.create_subscription(String, '/science_routine_status_update', self.update_routine_status_window, 10 )

    def init_timer_tasks(self):
        # Initialize the routine status query timer
        self.routine_status_timer = self.create_routine_status_timer(self.routine_status_query_rate_ms)

    def update_routine_status_timer(self, polling_rate_ms):
        if self.routine_status_query_rate_ms != polling_rate_ms:
            # If it is a different rate
            self.routine_status_query_rate_ms = polling_rate_ms
            if self.routine_status_timer is not None:
                # Destroy the existing timer if it exists
                self.routine_status_timer.destroy()
            self.routine_status_timer = self.create_routine_status_timer(polling_rate_ms)

    def create_routine_status_timer(self, period_ms):
        return self.create_timer(period_ms / 1000, lambda: self.perform_routine_status_query())

    def update_actuator_state(self, msg: ScienceActuatorState):
        # Update the GUI with the new actuator state
        # This function will be called whenever a new ScienceActuatorState message is received

        self.update_state_table(msg)
        self.update_graphics(msg)
        self.actuator_poll_manager.update_attention(msg.index, msg.control)
        # self.get_logger().info(f"Actuator {msg.index} state updated: Position={msg.position}, Control={msg.control}, Reserved={msg.reserved}")

    def update_state_table(self, msg: ScienceActuatorState):
        # Update the table row corresponding to the actuator index
        table = self.qt.actuator_state_table
        if table.rowCount() <= msg.index:
            table.setRowCount(msg.index + 1)

        # Update the table cells for the actuator
        table.setItem(msg.index, 0, QtWidgets.QTableWidgetItem(f"{msg.position}"))
        table.setItem(msg.index, 1, QtWidgets.QTableWidgetItem(f"{msg.control}"))
        table.setItem(msg.index, 2, QtWidgets.QTableWidgetItem(f"{msg.reserved}"))

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

        self.qt.update() # Redraw the GUI to reflect the changes

    def update_routine_status_window(self, msg: String):
        """
        Callback function to update the QLabel with the received String message.
        """
        self.qt.routine_status_window.setText(msg.data)

class PollingTimer:
    def __init__(self, node, period_ms, func):
        self.node = node
        self.period_ms = period_ms
        self.timer = None
        self.func = func
        self.build()

    def build(self):
        if self.timer is not None:
            self.timer.destroy()
        self.timer = self.node.create_timer(self.period_ms / 1000, self.func)
        # self.node.get_logger().info(f"Timer created with period: {self.period_ms} ms")

    def change_rate(self, new_period_ms):
        if new_period_ms != self.period_ms:
            self.period_ms = new_period_ms
            self.build()

class ActuatorStatePoll(PollingTimer):
    def __init__(self, node, publisher, actuator_index, passive_rate_ms=1000, active_rate_ms=100):
        super().__init__(node, passive_rate_ms, lambda: publisher.publish(UInt8(data=actuator_index)))
        self.actuator_index = actuator_index
        self.passive_rate_ms = passive_rate_ms
        self.active_rate_ms = active_rate_ms
        self.is_active = False

    def set_active(self, active):
        if active:
            self.change_rate(self.active_rate_ms)
        else:
            self.change_rate(self.passive_rate_ms)

    def refresh(self):
        self.set_active(self.is_active)  # Update the timer with any new rates

    def update_passive_rate(self, new_rate):
        self.passive_rate_ms = new_rate
        self.refresh()  # Update the timer with the new passive rate

    def update_active_rate(self, new_rate):
        self.active_rate_ms = new_rate
        self.refresh()  # Update the timer with the new active rate

class ActuatorStatePollManager:
    def __init__(self, polls, passive_rate_ms=1000, active_rate_ms=100):
        self.polls = polls
        self.passive_rate_ms = passive_rate_ms
        self.active_rate_ms = active_rate_ms

    def update_attention(self, index, control):
        for poll in self.polls:
            if poll.actuator_index == index:
                poll.set_active(control != 0)
        # print(f"Actuator {index} attention updated: {control != 0}")
    
    def update_passive_rate_ms(self, new_rate):
        self.passive_rate_ms = new_rate
        for poll in self.polls:
            poll.update_passive_rate(new_rate)

    def update_active_rate_ms(self, new_rate):
        self.active_rate_ms = new_rate
        for poll in self.polls:
            poll.update_active_rate(new_rate)

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