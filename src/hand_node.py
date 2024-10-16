#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from dextrous_hand.utils import message_to_matrix
from dextrous_hand.Hand import HAND

class HandNode(Node):
    """
    Node to control the hand
    """

    POSITION_UPDATE_FREQUENCY = 64 # Hz

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

        self.timer = self.create_timer(1.0 / self.POSITION_UPDATE_FREQUENCY, self.main)


    def main(self):
        HAND.update_motor_positions()
        print(HAND)

    def finger_positions_callback(self, msg):
        joint_matrix = message_to_matrix(msg, (5, 3))
        HAND.set_fingers(joint_matrix)
        HAND.write_motor_targets()

    def wrist_position_callback(self, msg):
        HAND.set_wrist(msg.data)
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