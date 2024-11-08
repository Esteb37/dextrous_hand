#!/usr/bin/env python3

import time
import rclpy
import numpy as np
from rclpy.node import Node
from pynput import keyboard # type: ignore
from std_msgs.msg import Float32MultiArray

from dextrous_hand.Hand import HAND
import dextrous_hand.utils.ids as ids
from dextrous_hand.utils import constants
from dextrous_hand.utils.HandConfig import HandConfig
from dextrous_hand.motors.DynamixelClient import DynamixelClient

class TeleopNode(Node):
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
        super().__init__('teleop_node')
        self.config_publisher = self.create_publisher(Float32MultiArray, 'hand_config', 10)

        self.get_logger().info('teleop node started')
        self.subsystem_id = ids.SUBSYSTEMS.PINKY
        self.joint_id = 0

        # Start a keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.all = False

        if constants.GLOBAL_CONSTANTS["STARTUP_MODE"] == ids.STARTUP.LAST.name:

            self.get_logger().info('Reading current angles...')
            self.motor_bridge = DynamixelClient()

            self.motor_bridge.connect()

            self.motor_bridge.update_positions()

            self.get_logger().info('Disconnecting...')

            self.motor_bridge.disconnect()

            while self.motor_bridge.is_connected:
                self.motor_bridge.disconnect()
                time.sleep(0.1)

            self.get_logger().info('Disconnected')

            # Just the first three columns
            self.hand_config = HAND.get_config()

        elif constants.GLOBAL_CONSTANTS["STARTUP_MODE"] == ids.STARTUP.CUSTOM.name:
            self.hand_config = HandConfig("HOME")

        else:
            self.hand_config = HandConfig.default()

        # Run the publishing loop
        self.run()


    def on_press(self, key):
        try:
            if key.char in "qwertyu123oplk":
                # Map keys to finger ids
                if key.char == 'q':
                    self.subsystem_id = "PINKY"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'w':
                    self.subsystem_id = "RING"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'e':
                    self.subsystem_id = "MIDDLE"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'r':
                    self.subsystem_id = "INDEX"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 't':
                    self.subsystem_id = "THUMB"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'y':
                    self.subsystem_id = "WRIST"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'p':
                    self.subsystem_id = "POSITION"
                    self.joint_id = 0
                    self.all = False
                elif key.char == 'o':
                    self.subsystem_id = "ORIENTATION"
                    self.joint_id = 0
                    self.all = False

                # Map keys to joint ids
                elif key.char in '123':
                    self.joint_id = int(key.char) - 1

                elif key.char == 'u':
                    self.all = True
                    self.joint_id = 0

                # Adjust finger positions based on arrow keys
                elif key.char == "l":
                    if self.all:
                        for finger_config in self.hand_config.FINGERS:
                            finger_config[self.joint_id] += 0.1
                    else:
                        if self.subsystem_id == "POSITION":
                            self.hand_config[self.subsystem_id][self.joint_id] += 0.01
                        else:
                            self.hand_config[self.subsystem_id][self.joint_id] += 0.1

                elif key.char == "k":
                    if self.all:
                        for finger_config in self.hand_config.FINGERS:
                            finger_config[self.joint_id] -= 0.1
                    else:
                        if self.subsystem_id == "POSITION":
                            self.hand_config[self.subsystem_id][self.joint_id] -= 0.01
                        else:
                            self.hand_config[self.subsystem_id][self.joint_id] -= 0.1

                if self.all:
                    for config in self.hand_config.FINGERS:
                        print(config)
                else:
                    print(self.subsystem_id, self.hand_config[self.subsystem_id])

                print()

            elif key.char in "zxcvbnm":
                if key.char == 'z':
                    config = "HOME"
                elif key.char == 'x':
                    config = "GRASP"
                elif key.char == 'c':
                    config = "ROCK"
                elif key.char == 'v':
                    config = "FINGER"
                elif key.char == 'b':
                    config = "LOVE"
                elif key.char == 'n':
                    config = "THUMB"
                elif key.char == 'm':
                    config = "SPOCK"

                print(config)
                self.hand_config = HandConfig(config)

            if key.char == "a":
                angle = (np.sin(time.time()) * np.pi + np.pi) / 2
                self.hand_config = HandConfig(
                    PINKY=[0.0, 0.0, angle],
                    RING=[0.0, 0.0, angle],
                    MIDDLE=[0.0, 0.0, angle],
                    INDEX=[0.0, 0.0, angle],
                    THUMB=[0.0, 0.0, angle],
                )
                print(angle)

        except AttributeError:
            pass  # Handle special keys or other exceptions

        print(self.hand_config)

    def run(self):
        while rclpy.ok():
            # Publish the current finger positions
            self.config_publisher.publish(self.hand_config.as_msg())
            time.sleep(1.0 / constants.NODE_FREQUENCY_HZ)

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
