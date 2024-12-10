#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from ublox_read_2.msg import PositionVelocityTime


class PositionVelocityTimeTranslator(Node):
    """
    Description: 
    Node that takes in ublox PositionVelocityTime messages and sends out
    NavSatFix messages that are used by the ukf navsat transform node.
    """
    def __init__(self):
        super().__init__('position_velocity_time_translator')

        # Subscribers
        self.ublox_subscription = self.create_subscription(PositionVelocityTime, 'PosVelTime', self.ublox_callback, 10)  # Creates a subscriber to ublox node
        
        # Publishers
        self.lla_publisher = self.create_publisher(NavSatFix, '/ins/lla', 10)  # Creates a publisher to NavSatFix

    def ublox_callback(self, message):
        h_var = self.ublox_accuracy_to_variance(message.hAcc)  # Horizontal covariance
        v_var = self.ublox_accuracy_to_variance(message.vAcc)  # Vertical covariance

        # fixMe
        # The following line is a sanity test, I will comment it out/delete it after I am sure it
        # is right
        # assert message.pDOP == sqrt(2 * h_var ** 2 + v_var ** 2), "pDOP:{}, my pDOP: {}".format(
        #     message.pDOP, sqrt(2 * h_var ** 2 + v_var ** 2))

        # position_covariance is a measure of the accuracy of the gps reading.
        # h_var is for x and y, v_var is for z
        position_covariance = [
            h_var, 0, 0,
            0, h_var, 0,
            0, 0, v_var,
        ]

        # Creates navsat message
        navsatfix_message = NavSatFix(
            header=message.header,
            latitude=message.lla[0],
            longitude=message.lla[1],
            altitude=message.lla[2],
            position_covariance=position_covariance,
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
        )

        navsatfix_message.header.frame_id = 'gps_frame'  # Adds information to the navsat message header

        self.lla_publisher.publish(navsatfix_message)  # publishes message using publisher

    @staticmethod
    def ublox_accuracy_to_variance(acc):
        """Converts the accuracy given from the UBlox chip to variance 

        Arguments:
            acc {float} -- The accuracy given from the UBlox chip

        Returns:
            float -- The variance of from the given accuracy
        """
        # TODO Verify this procedure. The docs don't explain exactly what the 'accuracy' is, but this is assuming
        #  it is standard deviation
        acc_in_meters = acc * 1000
        variance = acc_in_meters * acc_in_meters
        return variance


def main(args=None):
    rclpy.init(args=args)
    position_velocity_time_translator = PositionVelocityTimeTranslator()
    rclpy.spin(position_velocity_time_translator)
    position_velocity_time_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
