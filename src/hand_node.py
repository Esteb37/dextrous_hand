#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from dextrous_hand.Hand import HAND

class HandNode(Node):
    """
    Node to control the hand
    """

    NODE_FREQUENCY_HZ = 60

    def __init__(self):
        super().__init__('hand_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'finger_positions',
            self.finger_positions_callback,
            10)

        self.timer = self.create_timer(1.0 / self.NODE_FREQUENCY_HZ, self.main)

        self.get_logger().info('Hand node started')

    def main(self):
        HAND.update_motor_positions()
        print(HAND)

    def finger_positions_callback(self, msg):
        HAND.set_fingers(msg.data)
        HAND.write_motor_targets()

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()