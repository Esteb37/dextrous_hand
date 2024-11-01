#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dextrous_hand.Hand import HAND
from dextrous_hand.HandConfig import HandConfig
from dextrous_hand.constants import NODE_FREQUENCY_HZ, MANUAL_CONTROL
from dextrous_hand.DynamixelClient import DynamixelClient
import time
import threading

class HandNode(Node):
    """
    Node to control the hand.
    Writing is controlled by the main thread, because it has extremely high frequency and is mostly only limited by the serial communication speed.
    Reading is controlled by a separate thread, because it has a lower frequency (around 64 hz) and as to not interfere with the writing thread.
    """

    def __init__(self):
        super().__init__('hand_node')

        self.motor_bridge = DynamixelClient()

        if MANUAL_CONTROL:
            self.motor_bridge.connect()
            self.motor_bridge.disable_torque()
            self.get_logger().info("Manual control enabled. Torque disabled.")
            self.initialized = True

        else:
            self.config_subscription = self.create_subscription(
                Float32MultiArray,
                'hand_config',
                self.hand_config_callback,
                100)

            self.initialized = False

        self.write_timer = self.create_timer(1.0 / NODE_FREQUENCY_HZ, self.write)

        # Start the read thread
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.get_logger().info('Hand node started')

    def hand_config_callback(self, msg):
        HAND.set_config(HandConfig.from_msg(msg))

        if not self.initialized:
            self.motor_bridge.connect()
            self.initialized = True

    def write(self):
        if self.initialized:
            self.motor_bridge.write_targets()

    def read_loop(self):
        """
        To be called in a separate thread
        """
        while rclpy.ok():
            self.read()
            time.sleep(1 / NODE_FREQUENCY_HZ)

    def read(self):
        if self.initialized:
            self.motor_bridge.update_positions()
            print(HAND)


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