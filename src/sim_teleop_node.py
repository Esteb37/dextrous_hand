#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from pynput import keyboard  # type: ignore
import time

from dextrous_hand.utils import matrix_to_message
import dextrous_hand.ids as ids
from dextrous_hand.Finger import finger_index
from dextrous_hand.constants import NODE_FREQUENCY_HZ


class SimTeleopNode(Node):
    """
    Node to send commands to the hand
    Controls:
    q: pinky
    w: ring
    e: middle
    r: index
    t: thumb
    y: wrist
    u: all fingers
    1: joint 1
    2: joint 2
    3: joint 3
    o: increase joint angle
    l: decrease joint angle
    """

    def __init__(self):
        super().__init__('sim_teleop_node')
        self.fingers_publisher = self.create_publisher(Float32MultiArray, 'finger_positions', 10)
        self.wrist_publisher = self.create_publisher(Float32, 'wrist_position', 10)

        self.get_logger().info('sim_teleop node started')
        self.finger_positions = np.zeros((5, 3))
        self.wrist_position = 0.0
        self.subsystem_id = ids.SUBSYSTEMS.PINKY
        self.joint_id = 0
        self.all = False

        # Start a keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Run the publishing loop
        self.run()

    def on_press(self, key):
        try:
            # Map keys to finger ids
            if key.char == 'q':
                self.subsystem_id = ids.SUBSYSTEMS.PINKY
                self.joint_id = 0
                self.all = False
            elif key.char == 'w':
                self.subsystem_id = ids.SUBSYSTEMS.RING
                self.joint_id = 0
                self.all = False
            elif key.char == 'e':
                self.subsystem_id = ids.SUBSYSTEMS.MIDDLE
                self.joint_id = 0
                self.all = False
            elif key.char == 'r':
                self.subsystem_id = ids.SUBSYSTEMS.INDEX
                self.joint_id = 0
                self.all = False
            elif key.char == 't':
                self.subsystem_id = ids.SUBSYSTEMS.THUMB
                self.joint_id = 0
                self.all = False
            elif key.char == 'y':
                self.subsystem_id = ids.SUBSYSTEMS.WRIST
                self.joint_id = 0
                self.all = False

            # Map keys to joint ids
            elif key.char in '123':
                self.joint_id = int(key.char) - 1

            elif key.char == 'u':
                self.all = True
                self.joint_id = 0

            # Adjust finger positions based on keys
            elif key.char == "o":
                if self.all:
                    for i in range(5):
                        self.finger_positions[i][self.joint_id] += 0.1
                else:
                    if self.subsystem_id == ids.SUBSYSTEMS.WRIST:
                        self.wrist_position += 0.1
                    else:
                        self.finger_positions[finger_index(self.subsystem_id)][self.joint_id] += 0.1

            elif key.char == "l":
                if self.all:
                    for i in range(5):
                        self.finger_positions[i][self.joint_id] -= 0.1
                else:
                    if self.subsystem_id == ids.SUBSYSTEMS.WRIST:
                        self.wrist_position -= 0.1
                    else:
                        self.finger_positions[finger_index(self.subsystem_id)][self.joint_id] -= 0.1

            print("ALL" if self.all else self.subsystem_id.name,
                  self.wrist_position if self.subsystem_id == ids.SUBSYSTEMS.WRIST else
                  self.finger_positions[finger_index(self.subsystem_id)])
            print()

        except AttributeError:
            pass  # Handle special keys or other exceptions

    def run(self):
        while rclpy.ok():
            # Publish the current finger positions
            self.fingers_publisher.publish(matrix_to_message(self.finger_positions))
            self.wrist_publisher.publish(Float32(data=self.wrist_position))
            time.sleep(1.0 / NODE_FREQUENCY_HZ)

def main(args=None):
    rclpy.init(args=args)
    node = SimTeleopNode()
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
