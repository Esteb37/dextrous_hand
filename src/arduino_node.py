#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adjust message type if needed
import serial

class ArduinoPublisher(Node):
    def __init__(self):
        super().__init__('arduino_publisher')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received: {data}')
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()