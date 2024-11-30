#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32

from dextrous_hand.Hand import HAND
from dextrous_hand.utils.HandConfig import HandConfig
from dextrous_hand.motors.DynamixelClient import DynamixelClient
from dextrous_hand.utils.constants import NODE_FREQUENCY_HZ, GLOBAL_CONSTANTS

class HandNode(Node):
    """
    Node to control the hand.
    Writing is controlled by the main thread, because it has extremely high frequency and is mostly only limited by the serial communication speed.
    Reading is controlled by a separate thread, because it has a lower frequency (around 64 hz) and as to not interfere with the writing thread.
    """

    def __init__(self):
        super().__init__('hand_node')

        self.declare_parameter("is_simulation", False)
        self.declare_parameter("manual_control", False)

        # These HAVE to be set before everything else so that all subsystems and the dynamixel client understand if they are in simulation mode
        GLOBAL_CONSTANTS["IS_SIMULATION"] = self.get_parameter("is_simulation").value
        GLOBAL_CONSTANTS["MANUAL_CONTROL"] = self.get_parameter("manual_control").value

        self.motor_bridge = DynamixelClient()

        if GLOBAL_CONSTANTS["MANUAL_CONTROL"]:
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
        self.print_timer = self.create_timer(0.1, self.print)

        # Start the read thread
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.arm_msg = HAND.read_current_config().pose_msg()
        self.arm_subscription = self.create_subscription(PoseStamped,'/franka/end_effector_pose',self.arm_config_callback,10)

        self.current_hand_config = self.create_publisher(Float32MultiArray, '/current_hand_config', 10)

        self.motor_target_publisher = self.create_publisher(Float32MultiArray, '/motors/target/positions', 10)
        self.motor_dxl_target_publisher = self.create_publisher(Float32MultiArray, '/motors/target/dxl_positions', 10)
        self.motor_read_publisher = self.create_publisher(Float32MultiArray, '/motors/read/positions', 10)
        self.motor_dxl_read_publisher = self.create_publisher(Float32MultiArray, '/motors/read/dxl_positions', 10)
        self.motor_current_read_publisher = self.create_publisher(Float32MultiArray, '/motors/read/dxl_currents', 10)
        self.motor_velocity_read_publisher = self.create_publisher(Float32MultiArray, '/motors/read/dxl_velocities', 10)
        self.motor_read_publishers = [self.motor_read_publisher, self.motor_dxl_read_publisher, self.motor_velocity_read_publisher, self.motor_current_read_publisher]

        self.current_publishers = [self.create_publisher(Float32, "/motors/current/"+motor.name, 10) for motor in self.motor_bridge.motors]

        self.get_logger().warn('Hand node started, waiting for hand config...')

    def hand_config_callback(self, msg):
        HAND.write_config(HandConfig.from_msg(msg))
        # TODO: edit config with the new wrist joint

        if not self.initialized:
            self.motor_bridge.connect()
            self.initialized = True

    def write(self):
        if self.initialized:
            self.motor_bridge.write_targets()

            for i, motor in enumerate(self.motor_bridge.motors):
                self.current_publishers[i].publish(Float32(data=motor.dxl_current))

    def read_loop(self):
        """
        To be called in a separate thread
        """
        while rclpy.ok():
            self.read()
            time.sleep(1 / NODE_FREQUENCY_HZ)

    def read(self):
        if self.initialized:
            # This needs to be called before calling read_config
            self.motor_bridge.update_positions()
            HAND.update_arm_pose(self.arm_msg)

            self.current_hand_config.publish(HAND.read_current_config().as_msg())

            motor_positions = Float32MultiArray()
            motor_positions.data = self.motor_bridge.read_positions()
            self.motor_read_publisher.publish(motor_positions)

            motor_dxl_positions = Float32MultiArray()
            motor_dxl_positions.data = self.motor_bridge.read_dxl_positions()
            self.motor_dxl_read_publisher.publish(motor_dxl_positions)

            motor_velocities = Float32MultiArray()
            motor_velocities.data = self.motor_bridge.read_velocities()
            self.motor_velocity_read_publisher.publish(motor_velocities)

            motor_currents = Float32MultiArray()
            motor_currents.data = self.motor_bridge.read_currents()
            self.motor_current_read_publisher.publish(motor_currents)

            motor_targets = Float32MultiArray()
            motor_targets.data = self.motor_bridge.read_targets()
            self.motor_target_publisher.publish(motor_targets)

            motor_dxl_targets = Float32MultiArray()
            motor_dxl_targets.data = self.motor_bridge.read_dxl_targets()
            self.motor_dxl_target_publisher.publish(motor_dxl_targets)


    def print(self):
        if self.initialized:
            print(HAND)


    def arm_config_callback(self, msg):
        self.arm_msg = msg

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