#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import (
    Elevator,
    ScienceLinearActuatorDirection,
    ScienceCacheDoor,
    ScienceAugerOn,
    ScienceSwitchTool,
    ScienceSaveSecondaryCache,
    ScienceSecondaryCachePosition
)

# Left joy vertical -> elevator X
# Right joy vertical -> linear actuator X
# Bumpers -> switch tool X
# Right trigger -> drill forward X
# Left trigger -> drill reverse X
# A button -> toggle linear actuator (trap door) X
# Back button -> toggle secondary cache out or in
# Xbox button -> move sample to secondary cache

A = 0  # Toggle trap door linear actuator
B = 1  # Emergency stop secondary cache automation
X = 2
Y = 3
LB = 4  # Switch actuator to previous
RB = 5  # Switch actuator to next
BACK = 6  # Toggle secondary cache out or in
START = 7
POWER = 8  # (Xbox) Move sample to secondary cache
BUTTON_STICK_LEFT = 9
BUTTON_STICK_RIGHT = 10

LEFT_STICK_HORIZONTAL = 0
LEFT_STICK_VERTICAL = 1  # Move elevator up/down
LT = 2  # Drill reverse
RIGHT_STICK_HORIZONTAL = 3
RIGHT_STICK_VERTICAL = 4  # Move linear actuator up/down
RT = 5  # Drill forward
DPAD_HORIZONTAL = 6
DPAD_VERTICAL = 7

CACHE_DOOR_OPEN = False
CACHE_DOOR_CLOSED = True


class XBOX(Node):
    def __init__(self):
        super().__init__("xbox_science")

        self.get_logger().info('running joystick!')

        # Subscribers
        self.sub_joy = self.create_subscription(
            Joy, "/joy_science_input", self.joy_callback, 10
        )

        # Publishers
        self.pub_science_la_direction = self.create_publisher(
            ScienceLinearActuatorDirection, "/science_la_direction", 10
        )
        self.pub_elevator = self.create_publisher(Elevator, "/elevator", 10)
        self.pub_primary_cache_door_position = self.create_publisher(
            ScienceCacheDoor, "/science_primary_cache_door_position", 10
        )
        self.pub_auger_on = self.create_publisher(ScienceAugerOn, "/science_auger_on", 10)
        self.pub_switch_tool_direction = self.create_publisher(
            ScienceSwitchTool, "/science_switch_tool", 10
        )
        self.pub_save_secondary_cache = self.create_publisher(
            ScienceSaveSecondaryCache, "/science_save_secondary_cache", 10
        )
        self.pub_stop_secondary_cache_automation = self.create_publisher(
            ScienceSaveSecondaryCache, "/science_stop_secondary_cache", 10
        )
        self.pub_move_secondary_cache = self.create_publisher(
            ScienceSecondaryCachePosition, "/science_move_secondary_cache", 10
        )

        # Initialize state variables
        self.linear_actuator_speed = 0
        self.elev_direction = 0
        self.primary_cache_door_position = CACHE_DOOR_CLOSED
        self.auger_speed = 0
        self.switch_tool_direction = 0

    def joy_callback(self, msg: Joy):
        DEAD_ZONE = 0.05

        # Linear Actuator Control: RIGHT JOYSTICK VERTICAL
        if msg.axes[RIGHT_STICK_VERTICAL] > DEAD_ZONE:
            linear_actuator_speed = msg.axes[RIGHT_STICK_VERTICAL]
        elif msg.axes[RIGHT_STICK_VERTICAL] < -DEAD_ZONE:
            linear_actuator_speed = msg.axes[RIGHT_STICK_VERTICAL]
        else:
            linear_actuator_speed = 0

        # Elevator Control: LEFT JOYSTICK VERTICAL
        if msg.axes[LEFT_STICK_VERTICAL] > DEAD_ZONE:
            elev_direction = 1
        elif msg.axes[LEFT_STICK_VERTICAL] < -DEAD_ZONE:
            elev_direction = -1
        else:
            elev_direction = 0

        # Primary Cache Door Control: A BUTTON
        primary_cache_door_position = self.primary_cache_door_position
        if msg.buttons[A]:
            primary_cache_door_position = not self.primary_cache_door_position

        # Auger Control: LEFT AND RIGHT TRIGGER
        if msg.axes[RT] < -DEAD_ZONE:
            auger_speed = -msg.axes[RT]
        elif msg.axes[LT] < -DEAD_ZONE:
            auger_speed = msg.axes[LT]
        else:
            auger_speed = 0

        # Switch Tool: RIGHT AND LEFT BUMPERS
        if msg.buttons[LB]:
            switch_tool_direction = -1
            primary_cache_door_position = CACHE_DOOR_CLOSED
        elif msg.buttons[RB]:
            switch_tool_direction = 1
            primary_cache_door_position = CACHE_DOOR_CLOSED
        else:
            switch_tool_direction = 0

        # Save to secondary cache: XBOX Button
        if msg.buttons[POWER]:
            self.pub_save_secondary_cache.publish(ScienceSaveSecondaryCache(True))

        # Emergency stop secondary cache automation
        if msg.buttons[B]:
            self.pub_stop_secondary_cache_automation.publish(ScienceSaveSecondaryCache(False))

        # Move secondary cache out and in
        if msg.buttons[BACK]:
            self.pub_move_secondary_cache.publish(ScienceSecondaryCachePosition(1))

        # Publish all changed controls
        if linear_actuator_speed != self.linear_actuator_speed:
            self.linear_actuator_speed = int(linear_actuator_speed * 127)
            self.pub_science_la_direction.publish(
                ScienceLinearActuatorDirection(self.linear_actuator_speed)
            )

        if elev_direction != self.elev_direction:
            self.elev_direction = elev_direction
            real_elev_direction = 1 if elev_direction > 0 else 0
            elev_speed = int(255 * 0.9 if elev_direction != 0 else 0)
            msg = Elevator()
            msg.elevator_direction = real_elev_direction
            msg.elevator_speed = elev_speed
            self.pub_elevator.publish(msg)

        if primary_cache_door_position != self.primary_cache_door_position:
            self.primary_cache_door_position = primary_cache_door_position
            self.pub_primary_cache_door_position.publish(ScienceCacheDoor(primary_cache_door_position))

        if auger_speed != self.auger_speed:
            self.auger_speed = int(auger_speed * 127)
            self.pub_auger_on.publish(ScienceAugerOn(self.auger_speed))

        if switch_tool_direction != self.switch_tool_direction:
            self.switch_tool_direction = switch_tool_direction
            if switch_tool_direction != 0:
                self.pub_switch_tool_direction.publish(
                    ScienceSwitchTool(switch_tool_direction)
                )


def main(args=None):
    rclpy.init(args=args)
    xbox = XBOX()
    rclpy.spin(xbox)
    xbox.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
