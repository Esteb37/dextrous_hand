#!/usr/bin/env python3

import rclpy
import time
from ruamel.yaml import YAML

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dextrous_hand.DynamixelClient import DynamixelClient
from dextrous_hand.constants import GLOBAL_CONSTANTS
from dextrous_hand.utils import parent_dir

class CalibrateNode(Node):

    def __init__(self):
        super().__init__('calibrate_node')
        self.config_publisher = self.create_publisher(Float32MultiArray, 'hand_config', 10)

        self.get_logger().info('Calibration node started')

        if GLOBAL_CONSTANTS["IS_SIMULATION"]:
            self.get_logger().error("IS_SIMULATION must be set to False for calibration.")
            return

        self.get_logger().info('Calibrating zeros...')

        motor_bridge = DynamixelClient()

        motor_bridge.read_zeros()

        yaml = YAML()
        yaml.preserve_quotes = True  # Preserves quotes and structure

        yaml_path = parent_dir() + '/data/constants/motors.yaml'

        with open(yaml_path) as file:
            data = yaml.load(file)

        for motor in motor_bridge.motors:
            data[motor.name]["zero"] = float(motor.angle)
            motor.zero = motor.angle

        with open(yaml_path, 'w') as file:
            yaml.dump(data, file)

        print("Zero calibration done. Calibrate directions? (y/n)")
        response = input()
        if response == 'y':
            motor_bridge.disable_torque()
            print("Flex all the finger joints towards the inside of the palm and the wrist forward.")
            input()
            motor_bridge.read_directions()

            for motor in motor_bridge.motors:
                data[motor.name]["direction"] = motor.direction

        with open(yaml_path, 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info('File saved. Disconnecting...')

        motor_bridge.disconnect()

        while motor_bridge.is_connected:
            motor_bridge.disconnect()
            time.sleep(0.1)

        self.get_logger().info('Disconnected')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
