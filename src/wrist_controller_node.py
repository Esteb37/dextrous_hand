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
        self.wrist_difference_publisher = self.create_publisher(PoseStamped, 'wrist_difference', 10)

        self.wrist_controller = WristController()

        self.timer = self.create_timer(0.005, self.timer_publish_cb)

        self.get_logger().warn("Wrist controller node started")

    def wrist_cb(self, msg: PoseStamped):
        self.wrist_frame_id = msg.header.frame_id
        if self.wrist_pos is None or self.wrist_quat is None:
            self.wrist_pos, self.wrist_quat = pose_to_pos_orient(msg)
            self.wrist_pos_init = self.wrist_pos
            self.wrist_quat_init = self.wrist_quat
        else:
            self.wrist_pos, self.wrist_quat = pose_to_pos_orient(msg)

    def elbow_cb(self, msg: PoseStamped):
        self.eblow_frame_id = msg.header.frame_id
        self.elbow_pos, self.elbow_quat = pose_to_pos_orient(msg)

    def timer_publish_cb(self):
        if self.wrist_pos is None or self.wrist_quat is None or self.elbow_pos is None or self.elbow_quat is None:
            return
        # wrist_joint, wrist_x_quat, elbow_x_quat = self.wrist_controller.get_wrist_command(self.wrist_quat_init, self.wrist_quat, self.elbow_quat)
        wrist_joint, wrist_to_elbow_in_coil_frame_quat = self.wrist_controller.get_wrist_command(self.wrist_quat_init, self.wrist_quat, self.elbow_quat)

        wrist_msg = Float32()
        wrist_msg.data = wrist_joint

        # wrist_2_msg = Float32()
        # wrist_2_msg.data = wrist_joint2

        # wrist_3_msg = Float32()
        # wrist_3_msg.data = wrist_joint_3

        difference_quat = PoseStamped()
        difference_quat.header.frame_id = self.wrist_frame_id
        difference_quat.pose.position.x = self.wrist_pos[0]
        difference_quat.pose.position.y = self.wrist_pos[1]
        difference_quat.pose.position.z = self.wrist_pos[2]

        difference_quat.pose.orientation.x = wrist_to_elbow_in_coil_frame_quat[0]
        difference_quat.pose.orientation.y = wrist_to_elbow_in_coil_frame_quat[1]
        difference_quat.pose.orientation.z = wrist_to_elbow_in_coil_frame_quat[2]
        difference_quat.pose.orientation.w = wrist_to_elbow_in_coil_frame_quat[3]

        # elbow_x = PoseStamped()
        # elbow_x.header.frame_id = self.wrist_frame_id
        # elbow_x.pose.position.x = self.elbow_pos[0]
        # elbow_x.pose.position.y = self.elbow_pos[1]
        # elbow_x.pose.position.z = self.elbow_pos[2]

        # elbow_x.pose.orientation.x = elbow_x_quat[0]
        # elbow_x.pose.orientation.y = elbow_x_quat[1]
        # elbow_x.pose.orientation.z = elbow_x_quat[2]
        # elbow_x.pose.orientation.w = elbow_x_quat[3]        

        self.wrist_publisher.publish(wrist_msg)
        self.wrist_difference_publisher.publish(difference_quat)
        # self.elbow_x_publisher.publish(elbow_x)
        # self.wrist_2_publisher.publish(wrist_2_msg)
        # self.wrist_3_publisher.publish(wrist_3_msg)

def main(args=None):
    rclpy.init(args=args)
    wrist_controller_node = WristControllerNode()
    rclpy.spin(wrist_controller_node)
    wrist_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()