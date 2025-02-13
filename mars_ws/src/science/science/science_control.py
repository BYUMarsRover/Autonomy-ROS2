#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# from rover_msgs.msg import ( 
#     # ScienceAugerOn,
#     # ScienceToolPosition,
#     # ScienceLinearActuatorDirection,
#     # ScienceCacheDoor,
#     # ScienceSwitchTool,
#     # ScienceSaveSecondaryCache,
#     # ScienceSecondaryCachePosition
# )

# NUMBER_OF_TOOLS = 2
# CACHE_DOOR_OPEN = False
# CACHE_DOOR_CLOSED = True
# CACHE_ENGAGED = 1
# CACHE_DISENGAGED = 0
# AUGER_TOOL_UP = 127
# AUGER_TOOL_DOWN = -127
# DRILL_FULL_REVERSE = -127

# MOVE_SECONDARY_CACHE_TIME = 6
# MOVE_AUGER_TIME = 7
# OPEN_CLOSE_CACHE_TIME = 3
# DUMP_CACHE_TIME = 10
# MOVE_AUGER_DOWN_TIME = 1.2

# START_AUTOMATION = 0  # Move secondary cache in
# MOVE_AUGER_UP = 1
# OPEN_SECONDARY_CACHE = 2
# CLOSE_SECONDARY_CACHE = 3
# MOVE_SECONDARY_OUT = 4
# MOVE_ACTUATOR_DOWN = 5
# OPEN_PRIMARY_CACHE = 6
# CLOSE_PRIMARY_CACHE = 7
# FINISH_AUTOMATION = 8  # Move secondary cache in and reset everything

# class ScienceControl(Node):

#     def __init__(self):
#         super().__init__('science_control')
#         self.science_la_direction = self.create_subscription(ScienceLinearActuatorDirection, '/science_la_direction', self.la_direction_callback, 10)
#         self.science_primary_cache_door = self.create_subscription(ScienceCacheDoor, '/science_primary_cache_door_position', self.primary_cache_door_callback, 10)
#         self.science_auger_on = self.create_subscription(ScienceAugerOn, '/science_auger_on', self.auger_on_callback, 10)
#         self.science_switch_tool = self.create_subscription(ScienceSwitchTool, '/science_switch_tool', self.switch_tool_callback, 10)
#         self.science_save_secondary_cache = self.create_subscription(ScienceSaveSecondaryCache, '/science_save_secondary_cache', self.save_secondary_cache_callback, 10)
#         self.science_stop_secondary_cache = self.create_subscription(ScienceSaveSecondaryCache, '/science_stop_secondary_cache', self.stop_secondary_cache_automation_callback, 10)
#         self.science_move_secondary_cache = self.create_subscription(ScienceSecondaryCachePosition, '/science_move_secondary_cache', self.move_secondary_cache_callback, 10)

#         self.auger_val_publisher = self.create_publisher(ScienceAugerOn, '/science_serial_auger',10)
#         self.tool_position_publisher = self.create_publisher(ScienceToolPosition, '/science_tool_position', 10)
#         self.linear_actuator_publisher = self.create_publisher(ScienceLinearActuatorDirection, '/science_serial_la', 10)
#         self.primary_cache_door_publisher = self.create_publisher(ScienceCacheDoor,'/science_serial_primary_cache_door', 10)
#         self.secondary_cache_door_publisher = self.create_publisher(ScienceCacheDoor, '/science_serial_secondary_cache_door', 10)
#         self.secondary_cache_position_publisher = self.create_publisher(ScienceSecondaryCachePosition, '/science_serial_secondary_cache', 10)

#         self.current_tool_index = 0
#         self.la_direction = 0
#         self.primary_cache_door_position = CACHE_DOOR_CLOSED
#         self.secondary_cache_door_position = CACHE_DOOR_CLOSED
#         self.auger_speed = 0
#         self.secondary_cache_position = CACHE_DISENGAGED

#         self.timer = None
#         self.is_saving = False
#         self.current_state = START_AUTOMATION

#         hz = 60.

#         self.create_timer(1./hz, self.publish_controls)
#         self.create_timer(1./hz, self.run_secondary_cache_automation)

#     def la_direction_callback(self, msg: ScienceLinearActuatorDirection):
#         if not self.is_saving:
#             self.la_direction = msg.direction
#         else:
#             print('Secondary Cache automation currently running')
    
#     def primary_cache_door_callback(self, msg: ScienceCacheDoor):
#         if not self.is_saving:
#             self.primary_cache_door_position = msg.position
#         else:
#             print('Secondary Cache automation currently running')
    
#     def auger_on_callback(self, msg: ScienceAugerOn):
#         if not self.is_saving:
#             self.auger_speed = msg.auger_speed
#         else:
#             print('Secondary Cache automation currently running')

#     def switch_tool_callback(self, msg: ScienceSwitchTool):
#         if not self.is_saving:
#             self.current_tool_index = (self.current_tool_index + msg.direction) % 2
#         else:
#             print('Secondary Cache automation currently running')

#     def move_secondary_cache_callback(self, msg: ScienceSecondaryCachePosition):
#         if not self.is_saving:
#             if self.secondary_cache_position == CACHE_DISENGAGED:
#                 self.secondary_cache_position = CACHE_ENGAGED
#             else:
#                 self.secondary_cache_position = CACHE_DISENGAGED
#         else:
#             print('Secondary cache automation currently running')

#     def save_secondary_cache_callback(self, msg: ScienceSaveSecondaryCache):
#         if self.is_saving:
#             print('Saving to secondary cache already in progress')
#             return
#         else:
#             print('SAVING SECONDARY CACHE!!!')
#             self.reset_state_machine()
#             self.is_saving = True

#     def stop_secondary_cache_automation_callback(self, msg: ScienceSaveSecondaryCache):
#         if not self.is_saving:
#             print('Automation not running')
#             return
#         else:
#             print('Stopping the automation')
#             # stop the state machine
#             self.reset_state_machine()
#             self.auger_speed = 0
#             self.secondary_cache_position = CACHE_DISENGAGED
#             self.secondary_cache_door_position = CACHE_DOOR_CLOSED

#     def run_secondary_cache_automation(self):
#         if self.is_saving:
#             if self.timer is None:
#                 self.timer = rclpy.time.Time

#             print('state', self.current_state)
#             if self.current_state == START_AUTOMATION:
#                 self.primary_cache_door_position = CACHE_DOOR_CLOSED
#                 self.secondary_cache_position = CACHE_DISENGAGED
#                 self.switch_state(MOVE_SECONDARY_CACHE_TIME)
#             elif self.current_state == MOVE_AUGER_UP:
#                 self.la_direction = AUGER_TOOL_UP
#                 self.switch_state(MOVE_AUGER_TIME)
#             elif self.current_state == OPEN_SECONDARY_CACHE:
#                 self.secondary_cache_door_position = CACHE_DOOR_OPEN
#                 self.switch_state(OPEN_CLOSE_CACHE_TIME)
#             elif self.current_state == CLOSE_SECONDARY_CACHE:
#                 self.secondary_cache_door_position = CACHE_DOOR_CLOSED
#                 self.switch_state(OPEN_CLOSE_CACHE_TIME)
#             elif self.current_state == MOVE_SECONDARY_OUT:
#                 self.secondary_cache_position = CACHE_ENGAGED
#                 self.switch_state(MOVE_SECONDARY_CACHE_TIME)
#             elif self.current_state == MOVE_ACTUATOR_DOWN:
#                 self.la_direction = AUGER_TOOL_DOWN
#                 self.switch_state(MOVE_AUGER_DOWN_TIME)
#             elif self.current_state == OPEN_PRIMARY_CACHE:
#                 self.la_direction = 0
#                 self.primary_cache_door_position = CACHE_DOOR_OPEN
#                 self.auger_speed = DRILL_FULL_REVERSE
#                 self.switch_state(OPEN_CLOSE_CACHE_TIME + DUMP_CACHE_TIME)
#             elif self.current_state == CLOSE_PRIMARY_CACHE:
#                 self.auger_speed = 0
#                 self.primary_cache_door_position = CACHE_DOOR_CLOSED
#                 self.switch_state(OPEN_CLOSE_CACHE_TIME)
#             elif self.current_state == FINISH_AUTOMATION:
#                 self.secondary_cache_position = CACHE_DISENGAGED
#                 self.switch_state(MOVE_SECONDARY_CACHE_TIME)
#             else:
#                 print('Invalid automation state, resetting to the beginning')
#                 self.reset_state_machine()

#     def switch_state(self, time_difference):
#         difference = rclpy.time.Time - self.timer
#         print(difference)
#         if difference >= time_difference:
#             if self.current_state == FINISH_AUTOMATION:
#                 self.reset_state_machine()
#             else:
#                 self.current_state += 1
#                 self.timer = rclpy.time.Time

#     def reset_state_machine(self):
#         self.current_state = START_AUTOMATION
#         self.timer = None
#         self.is_saving = False

#     def publish_controls(self):

#         # Auger On
#         msg = ScienceAugerOn()
#         msg.auger_speed = self.auger_speed
#         self.auger_val_publisher.publish(msg)

#         # Linear Actuator Values
#         msg = ScienceLinearActuatorDirection()
#         msg.direction = self.la_direction
#         self.linear_actuator_publisher.publish(msg)

#         # Primary Cache Door
#         msg = ScienceCacheDoor()
#         msg.position = self.primary_cache_door_position
#         self.primary_cache_door_publisher.publish(msg)

#         # Tool Position
#         msg = ScienceToolPosition()
#         msg.position = self.current_tool_index
#         self.tool_position_publisher.publish(msg)

#         # Secondary Cache Door
#         msg = ScienceCacheDoor()
#         msg.position = self.secondary_cache_door_position
#         self.secondary_cache_door_publisher.publish(msg)

#         # Secondary Cache Position
#         msg = ScienceSecondaryCachePosition()
#         msg.secondary_cache_position = self.secondary_cache_position
#         self.secondary_cache_position_publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     science_control = ScienceControl()
#     science_control.get_logger().info("Science control online.")
#     rclpy.spin(science_control)
#     science_control.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    main()