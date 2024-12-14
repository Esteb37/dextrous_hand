#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

front_paths = ["big_blue/_oakd_front_view_color_1733998526041027072", "big_blue/_observations_images_oakd_front_view_color__home_esteb37_Downloads_synch_data_20241210_190907_pickup.h5", "big_blue/_observations_images_oakd_front_view_color__home_esteb37_Downloads_synch_data_20241210_213936_pickup.h5", "small_blue/_oakd_front_view_color_1733996905197364736", "big_red/_oakd_front_view_color_1734000573692312320"]
side_paths = ["big_blue/_oakd_side_view_color_1733999143838979840", "big_blue/_observations_images_oakd_side_view_color__home_esteb37_Downloads_synch_data_20241210_191013_pickup.h5", "big_blue/_observations_images_oakd_side_view_color__home_esteb37_Downloads_synch_data_20241210_191557_pickup.h5", "small_blue/_oakd_side_view_color_1734001590353735168", "big_red/_oakd_side_view_color_1733993085239463168"]

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        self.bridge = CvBridge()


        self.front_pub = self.create_publisher(Image, "/oakd_front_view/color", 10)
        self.side_pub = self.create_publisher(Image, "/oakd_side_view/color", 10)

        self.get_logger().warn("Camera node started")

        for i, front_path in enumerate(front_paths):
            front_path ="/home/esteb37/Downloads/images/" + front_path + ".jpg"
            side_path = side_paths[i]
            side_path = "/home/esteb37/Downloads/images/" + side_path + ".jpg"
            front_image = cv2.imread(front_path)
            side_image = cv2.imread(side_path)
            front_msg = self.bridge.cv2_to_imgmsg(front_image, "bgr8")
            side_msg = self.bridge.cv2_to_imgmsg(side_image, "bgr8")
            self.front_pub.publish(front_msg)
            self.side_pub.publish(side_msg)
            input("Press enter to continue")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
