#!/usr/bin/env python3

"""
ROS 2 node that uses the MotorInterface class to connect ROS graph to the hardware.
Also handles simulation.

Responsibilities:
- attempts to open connection with the motors.
    If unable, uses only simulation; otherwise, handles both
- receives commands on the /motor_command topic and accordingly commands the motors.
- periodically reads the positions of the motors and publishes them
"""

import motor_interface as mi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rover_msgs.msg import Elevator
import parameters as p
import utils as u


class MotorAndSimManager(Node):
    def __init__(self):
        super().__init__('motor_manager')

        self.joints = JointState()
        self.joints.position = p.INITIAL_Q
        self.joint_offset = [0.0] * p.N_JOINTS  # in INCREMENTS
        self.elev_pos = 0.75

        # SET UP MOTOR INTERFACE
        self.motors = None
        self.init_motors()

        # SUBSCRIBERS
        self.sub_motor_cmds = self.create_subscription(
            JointJog,
            "/motor_commands",
            self.send_cmds,
            qos_profile=p.SUBSCRIBER_QUEUE_SIZE
        )
        self.sub_reinit_motors = self.create_subscription(
            Empty,
            "/reinit_motor_connection",
            self._reinit_callback,
            qos_profile=p.SUBSCRIBER_QUEUE_SIZE
        )
        self.sub_reset_pos = self.create_subscription(
            Empty,
            "/reset_motor_positions",
            self._reset_pos_callback,
            qos_profile=p.SUBSCRIBER_QUEUE_SIZE
        )
        self.sub_elev = self.create_subscription(
            Elevator,
            "/elevator",
            self.update_elev,
            qos_profile=p.SUBSCRIBER_QUEUE_SIZE
        )

        # PUBLISHERS
        self.pub_arm_pos = self.create_publisher(JointState, "/arm_state", 1)
        self.pub_arm_pos_RVIZ = self.create_publisher(JointState, "/arm_elev_state", 1)

    # SUBSCRIBER CALLBACK FUNCTIONS

    def _reinit_callback(self, _):
        self.init_motors()

    def _reset_pos_callback(self, _):
        if self.active_connection:
            # Fetch radian positions from motors
            try:
                self.joint_offset = [
                    pos - iq / p.INCREMENTS_TO_RAD
                    for pos, iq in zip(self.motors.get_position_all(), p.INITIAL_Q)
                ]
                self.joints.position = p.INITIAL_Q
            except RuntimeError as RE:
                self.get_logger().info(str(RE))
                return
        else:
            self.joints.position = p.INITIAL_Q

    def send_cmds(self, cmds: JointJog):
        q_current = self.joints.position
        delta_q = [v * p.VEL_SCALE for v in cmds.velocities]
        new_q = [u.normalize_angle(qc + dq) for (qc, dq) in zip(q_current, delta_q)]

        valids, new_vs = u.enforce_joint_limits(
            new_q,
            cmds.velocities,
            p.DEFAULT_QLIM_BOTTOM,
            p.DEFAULT_QLIM_TOP,
        )

        if any([not v for v in valids]):
            self.get_logger().warn(
                "Nodes "
                + str([i + 1 for i, val in enumerate(valids) if not val])
                + " at joint limit. Zeroing commanded velocity."
            )

        if self.active_connection:
            self.motors.PVM_move(new_vs)
        else:
            self.joints.position = [
                nq if v else qc for nq, v, qc in zip(new_q, valids, q_current)
            ]

    def update_elev(self, elevator_cmd):
        dir = 1 if elevator_cmd.elevator_direction else -1
        speed = dir * elevator_cmd.elevator_speed
        self.elev_pos += p.ELEV_VEL_SCALE * speed

    def update_positions(self):
        if self.active_connection:
            try:
                received_pos = self.motors.get_position_all()
                offset_pos = [
                    pos - off for pos, off in zip(received_pos, self.joint_offset)
                ]
                self.joints.position = [
                    u.normalize_angle(pos * p.INCREMENTS_TO_RAD) for pos in offset_pos
                ]
            except RuntimeError as RE:
                self.get_logger().info(str(RE))
                return
        self.publish()

    def publish(self):
        self.pub_arm_pos.publish(JointState(position=self.joints.position, name=p.JOINT_NAMES[1:]))
        RVIZ_pos = [self.elev_pos] + [
            pos if i > 2 else -pos for i, pos in enumerate(self.joints.position)
        ]
        self.pub_arm_pos_RVIZ.publish(JointState(position=RVIZ_pos, name=p.JOINT_NAMES))

    def init_motors(self):
        if self.motors is not None:
            try:
                self.motors.close_connection()
            except RuntimeError as RE:
                self.active_connection = False
                self.get_logger().info("Unable to close connection to arm motors.")
                self.get_logger().info(str(RE))
        del self.motors
        self.motors = None

        try:
            self.motors = mi.MotorInterface()
            self.active_connection = True
        except RuntimeError as RE:
            self.motors = None
            self.active_connection = False
            self.get_logger().info("Unable to open connection to arm motors.")
            self.get_logger().info(str(RE))


def main(args=None):
    rclpy.init(args=args)
    manager = MotorAndSimManager()

    rate = manager.create_rate(p.CHECK_METHOD_HZ)
    while rclpy.ok():
        manager.update_positions()
        rclpy.spin_once(manager)
        rate.sleep()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
