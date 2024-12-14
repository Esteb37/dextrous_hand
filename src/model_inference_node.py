#!/bin/env python3

import rclpy
import numpy as np
from time import sleep
from copy import deepcopy
from threading import Lock
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Int32
from geometry_msgs.msg import PoseStamped

import torch
from dextrous_hand.mano.utils import numpy_to_float32_multiarray, float32_multiarray_to_numpy
from srl_il.export.il_policy import get_policy_from_ckpt

from dextrous_hand.utils.constants import GLOBAL_CONSTANTS
from dextrous_hand.utils.utils import parent_dir

class CameraListener(Node):
    def __init__(self, camera_topic, name, node):
        self.camera_topic = camera_topic
        self.lock = Lock()
        self.image = None
        self.name = name
        self.im_subscriber = node.create_subscription(
            Image, self.camera_topic, self.recv_im, 10
        )

    def recv_im(self, msg: Image):
        with self.lock:
            self.image = msg

    def get_im(self):
        with self.lock:
            return deepcopy(self.image)

class PolicyPlayerAgent(Node):
    def __init__(self):
        super().__init__("policy_publisher")

        self.declare_parameter("camera_topics", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("camera_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("big_policy_ckpt_path", GLOBAL_CONSTANTS["BIG_MODEL_PATH"])
        self.declare_parameter("small_policy_ckpt_path", GLOBAL_CONSTANTS["SMALL_MODEL_PATH"])

        self.declare_parameter("hand_qpos_dim", 17) # The dimension of the hand_qpos, we need this because we need to broadcast an all zero command to the hand at the beginning
        self.camera_topics = self.get_parameter("camera_topics").value
        self.camera_names = self.get_parameter("camera_names").value
        self.big_policy_ckpt_path = self.get_parameter("big_policy_ckpt_path").value
        self.small_policy_ckpt_path = self.get_parameter("small_policy_ckpt_path").value
        self.hand_qpos_dim = self.get_parameter("hand_qpos_dim").value

        if self.camera_topics is None or self.camera_names is None:
            raise ValueError("No camera topics or names provided")

        if self.big_policy_ckpt_path is None or self.big_policy_ckpt_path == "":
            raise ValueError("No big policy ckpt path provided")
        else:
            self.big_policy_ckpt_path = parent_dir() + "/data/models/" + self.big_policy_ckpt_path

        if self.small_policy_ckpt_path is None or self.small_policy_ckpt_path == "":
            raise ValueError("No small policy ckpt path provided")
        else:
            self.small_policy_ckpt_path = parent_dir() + "/data/models/" + self.small_policy_ckpt_path

        if self.hand_qpos_dim is None:
            raise ValueError("No hand qpos dimension provided")

        self.lock = Lock()

        self.hand_pub = self.create_publisher(
            Float32MultiArray, "/motors/target/positions", 10
        )
        self.hand_sub = self.create_subscription(
            Float32MultiArray, "/motors/read/positions", self.hand_callback, 10
        )

        self.arm_publisher = self.create_publisher(
            PoseStamped, "/franka/end_effector_pose_cmd", 10
        )
        self.arm_subscriber = self.create_subscription(
            PoseStamped, "/franka/end_effector_pose", self.arm_pose_callback, 10
        )

        self.camera_listeners = [
            CameraListener(camera_topic, camera_name, self)
            for camera_topic, camera_name in zip(self.camera_topics, self.camera_names)
        ]

        self.yolo_sub = self.create_subscription(Int32, "/yolo_decision", self.yolo_callback, 10)

        self.size_sub = self.create_subscription(String, "/cube_size", self.size_callback, 10)

        self.bridge = CvBridge()
        self.current_wrist_state = None
        self.current_hand_state = None

        self.big_policy = get_policy_from_ckpt(self.big_policy_ckpt_path)
        self.big_policy.reset_policy()
        self.small_policy = get_policy_from_ckpt(self.small_policy_ckpt_path)
        self.small_policy.reset_policy()
        self.policy_run = self.create_timer(0.001, self.run_policy_cb) # 20hz

        hand_msg = numpy_to_float32_multiarray(np.zeros(self.hand_qpos_dim))
        self.hand_pub.publish(hand_msg)

        self.yolo_decision = None
        self.cube_size = "big"

        self.get_logger().warn(f"Inference node started with models from {self.big_policy_ckpt_path} and {self.small_policy_ckpt_path}")
        self.start_time = self.get_clock().now()

    def yolo_callback(self, msg: Int32):
        self.yolo_decision = msg.data

    def size_callback(self, msg: String):
        self.cube_size = msg.data

    def publish(self, wrist_policy: np.ndarray, hand_policy: np.ndarray):
        # publish hand policy
        hand_msg = numpy_to_float32_multiarray(hand_policy)
        self.hand_pub.publish(hand_msg)

        # publish wrist policy
        wrist_msg = PoseStamped()
        wrist_msg.pose.position.x, wrist_msg.pose.position.y, wrist_msg.pose.position.z = wrist_policy[:3].astype(np.float64)
        (   wrist_msg.pose.orientation.x,
            wrist_msg.pose.orientation.y,
            wrist_msg.pose.orientation.z,
            wrist_msg.pose.orientation.w,
        ) = wrist_policy[3:].astype(np.float64)
        wrist_msg.header.stamp = self.get_clock().now().to_msg()
        wrist_msg.header.frame_id = "panda_link0"
        self.arm_publisher.publish(wrist_msg)

    def arm_pose_callback(self, msg: PoseStamped):
        current_wrist_state_msg = msg.pose
        position = [current_wrist_state_msg.position.x, current_wrist_state_msg.position.y, current_wrist_state_msg.position.z]
        quaternion = [current_wrist_state_msg.orientation.x, current_wrist_state_msg.orientation.y, current_wrist_state_msg.orientation.z, current_wrist_state_msg.orientation.w]
        self.current_wrist_state = np.concatenate([position, quaternion])

    def hand_callback(self, msg: Float32MultiArray):
        self.current_hand_state = float32_multiarray_to_numpy(msg)

    def get_current_observations(self):
        obs_dict = {}
        get_data_success = True

        images = {camera.name: camera.get_im() for camera in self.camera_listeners}
        if any([im is None for im in images.values()]):
            get_data_success = False
            print("Missing camera images", [im is not None for im in images.values()])
            return get_data_success, obs_dict

        images = {
            k: self.bridge.imgmsg_to_cv2(v, "bgr8").transpose(2, 0, 1)/255.0
            for k, v in images.items()
        }

        with self.lock:
            qpos_franka = self.current_wrist_state
            qpos_hand = self.current_hand_state
            yolo_decision = self.yolo_decision
        if qpos_franka is None or qpos_hand is None or yolo_decision is None:
            print("missing qpos_franka", qpos_franka is None)
            print("missing qpos_hand", qpos_hand is None)
            print("missing yolo_decision", yolo_decision is None)
            return False, obs_dict

        obs_dict.update(images)
        obs_dict['qpos_franka'] = qpos_franka
        obs_dict['qpos_hand'] = qpos_hand
        obs_dict['yolo_decision'] = self.yolo_decision
        return get_data_success, obs_dict

    def run_policy_cb(self):
        get_data_success, obs_dict = self.get_current_observations()
        if not get_data_success:
            self.get_logger().info("No observations available. Sleeping for 1 seconds.")
            sleep(1)
            return

        with torch.inference_mode():
            obs_dict = {k: torch.tensor(v).float().unsqueeze(0) for k, v in obs_dict.items()} # add batch dimension

            if self.cube_size == "big":
                actions = self.big_policy.predict_action(obs_dict)
            else:
                actions = self.small_policy.predict_action(obs_dict)

            wrist_action = actions["actions_franka"][0].cpu().numpy()
            hand_action = actions["actions_hand"][0].cpu().numpy()

        end_time = self.get_clock().now()
        self.publish(wrist_action, hand_action)
        self.get_logger().info(f"Policy: {self.cube_size} - {1/((end_time - self.start_time).nanoseconds/1 * 10**-9)} Hz")
        self.start_time = end_time

def main(args=None):
    rclpy.init(args=args)
    node = PolicyPlayerAgent()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
