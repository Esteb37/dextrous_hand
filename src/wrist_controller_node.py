#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from dextrous_hand.utils.utils import pose_to_pos_orient
from dextrous_hand.mano.wrist_controller import WristController

class WristControllerNode(Node):
    def __init__(self):
        super().__init__('wrist_controller_node')

        self.wrist_frame_id = None
        self.wrist_pos = None
        self.wrist_quat = None
        self.elbow_frame_id = None
        self.elbow_pos = None
        self.elbow_quat = None

        self.wrist_sub = self.create_subscription(
            PoseStamped,
            'ingress/wrist',
            self.wrist_cb,
            10)
        self.elbow_sub = self.create_subscription(
            PoseStamped,
            'ingress/elbow',
            self.elbow_cb,
            10)
        
        self.wrist_publisher = self.create_publisher(Float32, 'wrist_cmd', 10)

        self.wrist_controller = WristController()

        self.timer = self.create_timer(0.005, self.timer_publish_cb)

        self.get_logger().warn("Wrist controller node started")

    def wrist_cb(self, msg: PoseStamped):
        self.wrist_frame_id = msg.header.frame_id
        self.wrist_pos, self.wrist_quat = pose_to_pos_orient(msg)

    def elbow_cb(self, msg: PoseStamped):
        self.eblow_frame_id = msg.header.frame_id
        self.elbow_pos, self.elbow_quat = pose_to_pos_orient(msg)

    def timer_publish_cb(self):
        if self.wrist_pos is None or self.wrist_quat is None or self.elbow_pos is None or self.elbow_quat is None:
            return
        wrist_joint = self.wrist_controller.get_wrist_command(self.wrist_pos, self.wrist_quat, self.elbow_quat)

        wrist_msg = Float32()
        wrist_msg.data = wrist_joint

        self.wrist_publisher.publish(wrist_msg)

def main(args=None):
    rclpy.init(args=args)
    wrist_controller_node = WristControllerNode()
    rclpy.spin(wrist_controller_node)
    wrist_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()