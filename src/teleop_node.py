#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import numpy as np
from pynput import keyboard # type: ignore
import time

from dextrous_hand.utils import matrix_to_message
from dextrous_hand.Finger import FINGERS

class TeleopNode(Node):
    """
    Node to send commands to the hand
    Controls:
    q: pinky
    w: ring
    e: middle
    r: index
    f: thumb
    1: joint 1
    2: joint 2
    3: joint 3
    up: increase joint angle
    down: decrease joint angle
    """

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'finger_positions', 10)
        self.get_logger().info('teleop node started')
        self.finger_positions = np.zeros((5, 3))
        self.finger_id = 0
        self.joint_id = 0

        # Start a keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Run the publishing loop
        self.run()

    def on_press(self, key):
        try:
            # Map keys to finger ids
            if key.char == 'f':
                self.finger_id = 0
            elif key.char == 'r':
                self.finger_id = 1
            elif key.char == 'e':
                self.finger_id = 2
            elif key.char == 'w':
                self.finger_id = 3
            elif key.char == 'q':
                self.finger_id = 4

            # Map keys to joint ids
            elif key.char in '123':
                self.joint_id = int(key.char) - 1

            # Adjust finger positions based on arrow keys
            elif key.char == "o":
                self.finger_positions[self.finger_id][self.joint_id] += 0.01
            elif key.char == "l":
                self.finger_positions[self.finger_id][self.joint_id] -= 0.01

            print(FINGERS[self.finger_id].id.name, self.joint_id+1, self.finger_positions[self.finger_id][self.joint_id])

        except AttributeError:
            pass  # Handle special keys or other exceptions

    def run(self):
        while rclpy.ok():
            # Publish the current finger positions
            msg = matrix_to_message(self.finger_positions)
            self.publisher.publish(msg)
            time.sleep(0.1)  # Control publishing rate

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
