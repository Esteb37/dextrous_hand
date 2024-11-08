#!/usr/bin/env python3


import cv2
import time
import rclpy
import numpy as np
import tf2_ros as tf2
import mediapipe as mp
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from dextrous_hand.mano.Filters import LandmarksSmoothingFilter
from dextrous_hand.mano.retarget_utils import normalize_points_webcam

THUMB_IDX = 4
INDEX_FINGER_IDX = 8
VOLUME_UPDATE_INTERVAL = 15

class WebcamManoNode(Node):
    def __init__(self):
        super().__init__('rokoko_node')
        self.videoCap = cv2.VideoCapture(0)
        self.lastFrameTime = 0
        self.frame = 0
        self.handSolution = mp.solutions.hands # type: ignore
        self.hands = self.handSolution.Hands()
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.smoother = LandmarksSmoothingFilter(min_cutoff=1, beta=20, derivate_cutoff=10, disable_value_scaling=True)
        self.joint_pub = self.create_publisher(Float32MultiArray, '/ingress/mano', 10)

        # fixed frame broadcaster
        self.tf_broadcaster = tf2.StaticTransformBroadcaster(self)


    def timer_callback(self):
        self.broadcast_static_transform()

        self.frame += 1
        success, img = self.videoCap.read()
        if success:
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            thisFrameTime = time.time()
            fps = 1 / (thisFrameTime - self.lastFrameTime)
            self.lastFrameTime = thisFrameTime
            cv2.putText(img, f'FPS:{int(fps)}',
                        (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            recHands = self.hands.process(imgRGB)
            if recHands.multi_hand_landmarks:
                landmarks = recHands.multi_hand_landmarks[0].landmark

                float_landmarks = []

                for point in landmarks:
                    h, w, _ = img.shape
                    x, y = int(point.x * w), int(point.y * h)
                    cv2.circle(img, (x, y),
                                10, (255, 0, 255)
                                , cv2.FILLED)
                    float_landmarks.append([point.x, point.y, point.z])

                smooth_landmarks = self.smoother.apply(np.array(float_landmarks), 1)

                normalized_joint_pos = normalize_points_webcam(smooth_landmarks)


                self.publish_joints(normalized_joint_pos)

            cv2.imshow("CamOutput", img)
            cv2.waitKey(1)

    def publish_joints(self, joint_pos):
        arr = joint_pos

        msg = Float32MultiArray()
        msg.data = arr.flatten().tolist()

        # Set the 'layout' field of the message to describe the shape of the original array
        rows_dim = MultiArrayDimension()
        rows_dim.label = 'rows'
        rows_dim.size = arr.shape[0]
        rows_dim.stride = 1

        cols_dim = MultiArrayDimension()
        cols_dim.label = 'cols'
        cols_dim.size = arr.shape[1]
        cols_dim.stride = 1

        msg.layout.dim = [rows_dim, cols_dim]

        self.joint_pub.publish(msg)

    def broadcast_static_transform(self):
        transform = tf2.TransformStamped()

        # Set the header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"  # Or "map" or another fixed frame
        transform.child_frame_id = "camera"

        # Define translation (x, y, z) and rotation (quaternion: x, y, z, w)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.0  # Set a suitable height
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WebcamManoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()