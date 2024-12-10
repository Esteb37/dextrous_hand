#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from dextrous_hand.utils.constants import NODE_FREQUENCY_HZ, GLOBAL_CONSTANTS
from dextrous_hand.motors.DynamixelClient import DynamixelClient

class MotorsNode(Node):

    def __init__(self):
        super().__init__('motors_node')

        GLOBAL_CONSTANTS["IS_SIMULATION"] = True

        self.config_subscription = self.create_subscription(Float32MultiArray,'/motors/target/positions',self.motor_pos_callback, 10)

        self.config_publisher = self.create_publisher(Float32MultiArray,'/motors/read/positions', 10)

        self.motor_bridge = DynamixelClient()

        self.motor_bridge.connect()

        self.initialized = False

        self.write_timer = self.create_timer(1.0 / NODE_FREQUENCY_HZ, self.write)

        # Start the read thread
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.target_positions = Float32MultiArray()

        for motor in self.motor_bridge.motors:
            motor.target = 0

        self.get_logger().warn('Motors node started')

    def motor_pos_callback(self, msg):
        for i, motor in enumerate(self.motor_bridge.motors):
            motor.target = msg.data[i]

    def write(self):
        self.motor_bridge.write_targets()

    def read_loop(self):
        """
        To be called in a separate thread
        """
        while rclpy.ok():
            self.read()
            time.sleep(1 / NODE_FREQUENCY_HZ)

    def read(self):
        """
        To be called in a separate thread
        """
        self.motor_bridge.update_positions()
        positions = Float32MultiArray()
        positions.data = self.motor_bridge.read_positions()
        dim = MultiArrayDimension()
        dim.label = "motors"
        dim.size = len(self.motor_bridge.motors)
        positions.layout.dim = [dim]
        self.config_publisher.publish(positions)

def main(args=None):
    rclpy.init(args=args)
    node = MotorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()