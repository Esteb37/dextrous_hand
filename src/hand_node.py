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
    def __init__(self):
        super().__init__('hand_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'finger_positions',
            self.finger_positions_callback,
            10)

        self.get_logger().info('Hand node started')

    def finger_positions_callback(self, msg):
        HAND.set_fingers(msg.data)
        print(HAND)

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()