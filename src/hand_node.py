#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from Hand import HAND

class HandNode(Node):
    """
    Node to control the hand
    """
    def __init__(self):
        super().__init__('hand_node')
        HAND.print()

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()