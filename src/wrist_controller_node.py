#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from dextrous_hand.utils.utils import pose_to_pos_orient, pos_orient_to_pose
from dextrous_hand.mano.wrist_controller import WristController

class WristControllerNode(Node):
    def __init__(self):
        super().__init__('wrist_controller_node')

        self.arm_frame_id = None
        self.arm_pos = None
        self.arm_quat = None

        self.arm_cmd_sub = self.create_subscription(
            PoseStamped,
            'arm_end_effector_cmd',
            self.arm_cb,
            10)
        
        self.franka_publisher = self.create_publisher(PoseStamped, 'franka/end_effector_pose_cmd', 10)
        self.wrist_publisher = self.create_publisher(Float32, 'wrist_cmd', 10)
        # TODO 1: Create a simple WristController, sends all the wrist axis motion to the Franka / Hand
        # TODO 2: Subscribe to user input to allow for more custom control of the wrist between the Franka and Hand
        # Custom control : Either "wrist" (hand) priority, or "elbow" (franka) priority
        # Definition of priority: most of the motion will be done from the priority side, while the other side will complete the motion 
        # TODO 3: Allow for even more custom control than priority, by doing a percentage of the motion from the priority side, and the rest from the other side

        self.wrist_controller = WristController()
        # What would be in the WristController class?
        # we would ask the WristController to return the commands for the Franka and the Hand based on the user input
        # If we ignore any user input in the first implementation, we would only send the command to the wrist or franka depending on parameters
        # If we include user input, we would need to create a subscriber to the user input, and update the wrist_controller based on the user input
        # So wrist controller would have a method to update the priority based on the user input 



        self.timer = self.create_timer(0.005, self.timer_publish_cb)

        self.get_logger().warn("Wrist controller node started")

    def arm_cb(self, msg: PoseStamped):
        self.arm_frame_id = msg.header.frame_id
        self.arm_pos, self.arm_quat = pose_to_pos_orient(msg)
        # self.get_logger().info('Received: "%s"' % msg)


    def timer_publish_cb(self):
        if self.arm_pos is None or self.arm_quat is None:
            # self.get_logger().info('No arm pose received yet')
            return
        franka_pos_cmd, franka_quat_cmd, wrist_joint = self.wrist_controller.get_commands(self.arm_pos, self.arm_quat)
        franka_msg = pos_orient_to_pose(franka_pos_cmd, franka_quat_cmd)
        franka_msg.header.frame_id = self.arm_frame_id
        franka_msg.header.stamp = self.get_clock().now().to_msg()

        wrist_msg = Float32()
        wrist_msg.data = wrist_joint

        # self.get_logger().info('Publishing Franka: "%s"' % franka_msg)
        # self.get_logger().info('Publishing Wrist: "%s"' % wrist_msg.data)
        self.franka_publisher.publish(franka_msg)
        self.wrist_publisher.publish(wrist_msg)


def main(args=None):
    rclpy.init(args=args)
    wrist_controller_node = WristControllerNode()
    rclpy.spin(wrist_controller_node)
    wrist_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()