#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster, TransformStamped
from scipy.spatial.transform import Rotation as R

from dextrous_hand.mano.retargeter import Retargeter
from dextrous_hand.utils.HandConfig import HandConfig
from dextrous_hand.mano.utils import numpy_to_float32_multiarray
from dextrous_hand.mano.visualize_mano import ManoHandVisualizer
from dextrous_hand.mano.retarget_utils import rotation_matrix_x, rotation_matrix_y, rotation_matrix_z
from dextrous_hand.utils.constants import RETARGETER_PARAMS

class RetargeterNode(Node):
    def __init__(self, debug=False):
        super().__init__("rokoko_node")

        # start retargeter
        self.declare_parameter("retarget/mjcf_filepath", rclpy.Parameter.Type.STRING)
        self.declare_parameter("retarget/hand_scheme", rclpy.Parameter.Type.STRING)

        mjcf_filepath = self.get_parameter("retarget/mjcf_filepath").value
        self.hand_scheme = self.get_parameter("retarget/hand_scheme").value

        if mjcf_filepath is None:
            raise ValueError("No mjcf_filepath provided")

        if self.hand_scheme is None:
            raise ValueError("No hand_scheme provided")

        # subscribe to ingress topics
        self.ingress_mano_sub = self.create_subscription(
            Float32MultiArray, "/ingress/mano", self.ingress_mano_cb, 10
        )

        # TODO: Subscribe to a new topic for wrist pose, coming from the hijacker node
        self.ingress_wrist = self.create_subscription(
            PoseStamped, "/ingress/wrist", self.ingress_wrist_cb, 10
        )

        self.retargeter = Retargeter(urdf_filepath=mjcf_filepath, hand_scheme=self.hand_scheme)
        self.wrist_cmd_sub = self.create_subscription(
            Float32, "/wrist_cmd", self.wrist_cmd_cb, 10
        )


        self.joints_pub = self.create_publisher(
            Float32MultiArray, "/hand/policy_output", 10
        )

        self.hand_config_pub = self.create_publisher(Float32MultiArray, "/hand_config", 10)

        self.debug = True
        if self.debug:
            self.rviz_pub = self.create_publisher(MarkerArray, 'retarget/normalized_mano_points', 10)
            self.absolute_pub = self.create_publisher(MarkerArray, 'retarget/absolute_mano_points', 10)
            self.mano_hand_visualizer = ManoHandVisualizer(self.rviz_pub)
            self.absolute_mano_hand_visualizer = ManoHandVisualizer(self.absolute_pub)

        self.keypoint_positions = None
        self.wrist_position = np.array([0.0, 0.0, 0.0])
        self.wrist_orientation = [0.0, 0.0, 0.0, 1.0]
        self.wrist_initial_rotation = None
        self.wrist_initial_position = None
        self.wrist_joint_cmd = None

        self.timer = self.create_timer(0.005, self.timer_publish_cb)

        self.world_coil_broadcaster = TransformBroadcaster(self)

        self.get_logger().warn("Retargeter Node started")

    def ingress_mano_cb(self, msg):
        self.keypoint_positions = np.array(msg.data).reshape(-1, 3)

    def ingress_wrist_cb(self, msg):
        self.wrist_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.wrist_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def wrist_cmd_cb(self, msg):
        self.wrist_joint_cmd = float(msg.data)

    def timer_publish_cb(self):
        if self.keypoint_positions is None:
            return

        if self.debug:
            self.mano_hand_visualizer.reset_markers()
            self.absolute_mano_hand_visualizer.reset_markers()

        debug_dict = {}
        joint_angles, mano_fingertips, mano_mps, mano_palm, fingertips, mps, palm = self.retargeter.retarget(self.keypoint_positions, debug_dict)

        if self.debug:
            self.mano_hand_visualizer.generate_hand_markers(
                debug_dict["normalized_joint_pos"],
                stamp=self.get_clock().now().to_msg(),
            )

            self.mano_hand_visualizer.generate_keyvector_markers(
                mano_fingertips, mano_mps, mano_palm, stamp=self.get_clock().now().to_msg()
            )

            self.mano_hand_visualizer.generate_keyvector_markers(
                fingertips, mps, palm, detach = True, stamp=self.get_clock().now().to_msg()
            )

            self.absolute_mano_hand_visualizer.generate_hand_markers(
                debug_dict["original_joint_pos"],
                stamp=self.get_clock().now().to_msg(),
            )

        self.joints_pub.publish(
            numpy_to_float32_multiarray(np.deg2rad(joint_angles))
        )

        joint_rads = np.deg2rad(joint_angles)

        # if self.wrist_initial_position is None or self.wrist_initial_rotation is None:
        #     self.wrist_initial_position = self.wrist_position
        #     self.wrist_initial_rotation = R.from_quat(self.wrist_orientation)


        # wrist_rotation = (R.from_quat(self.wrist_orientation) * self.wrist_initial_rotation.inv()).as_quat().tolist()
        # wrist_position = (self.wrist_position - self.wrist_initial_position).tolist()

        if self.wrist_joint_cmd is None:
            wrist_joint = [0.0]
        else:
            wrist_joint = [self.wrist_joint_cmd]

        if self.hand_scheme == "hh":
            hand_config = HandConfig(WRIST = wrist_joint,
                                    PINKY = joint_rads[13:16],
                                    RING = joint_rads[10:13],
                                    MIDDLE = joint_rads[7:10],
                                    INDEX = joint_rads[4:7],
                                    THUMB = joint_rads[0:4],
                                    )
        else:
            hand_config = HandConfig(WRIST = wrist_joint,
                                    PINKY = joint_rads[12:15],
                                    RING = joint_rads[9:12],
                                    MIDDLE = joint_rads[6:9],
                                    INDEX = joint_rads[3:6],
                                    THUMB = joint_rads[0:3] + [0.0])

        self.hand_config_pub.publish(hand_config.as_msg())

        if self.debug:
            self.mano_hand_visualizer.publish_markers()
            self.absolute_mano_hand_visualizer.publish_markers()

        # Static transform from world to coil
        transform =  TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "coil"
        transform.child_frame_id = "world"

        transform.transform.translation.x = self.wrist_position[0]
        transform.transform.translation.y = self.wrist_position[1]
        transform.transform.translation.z = self.wrist_position[2]

        wrist_offset = rotation_matrix_x(0)
        wrist_offset = wrist_offset @ rotation_matrix_y(0)
        wrist_offset = wrist_offset @ rotation_matrix_z(0)

        wrist_offset = R.from_matrix(wrist_offset)

        # rotate self.wrist_orientation quaternion by the rotation matrix
        wrist_rot = R.from_quat(self.wrist_orientation)
        wrist_rot = wrist_offset * wrist_rot

        self.wrist_orientation = wrist_rot.as_quat()

        transform.transform.rotation.x = self.wrist_orientation[0]
        transform.transform.rotation.y = self.wrist_orientation[1]
        transform.transform.rotation.z = self.wrist_orientation[2]
        transform.transform.rotation.w = self.wrist_orientation[3]

        self.world_coil_broadcaster.sendTransform(transform)





def main(args=None):
    rclpy.init(args=args)
    node = RetargeterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
