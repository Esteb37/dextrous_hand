#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from dextrous_hand.utils import message_to_matrix
from dextrous_hand.Hand import HAND
from dextrous_hand.constants import NODE_FREQUENCY_HZ
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

        self.finger_subscription = self.create_subscription(
            Float32MultiArray,
            'finger_positions',
            self.finger_positions_callback,
            100)

        self.wrist_subscription = self.create_subscription(
            Float32,
            'wrist_position',
            self.wrist_position_callback,
            100)

        self.get_logger().info('Hand node started')

        self.write_timer = self.create_timer(1.0 / NODE_FREQUENCY_HZ, self.write)

        # Start the read thread
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def finger_positions_callback(self, msg):
        joint_matrix = message_to_matrix(msg, (5, 3))
        HAND.set_fingers(joint_matrix)

    def wrist_position_callback(self, msg):
        HAND.set_wrist(msg.data)

    def write(self):
        HAND.write_motor_targets()

    def read_loop(self):
        """
        To be called in a separate thread
        """
        while rclpy.ok():
            self.read()
            time.sleep(1 / NODE_FREQUENCY_HZ)

    def read(self):
        HAND.update_motor_positions()
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