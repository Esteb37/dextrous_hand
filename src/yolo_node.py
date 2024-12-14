#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2

from sensor_msgs.msg import Image

classes = ["big_blue", "big_yellow", "big_red", "small_blue", "small_yellow", "small_red", "tray_blue", "tray_yellow", "tray_red"]

class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.model = YOLO("/home/esteb37/ros2_ws/src/dextrous_hand/dextrous_hand/yolo/runs/detect/train7/weights/best.pt")

        self.front_sub = self.create_subscription(Image, "/oakd_front_view/color", self.front_callback, 10)
        self.side_sub = self.create_subscription(Image, "/oakd_side_view/color", self.side_callback, 10)

        self.front_pub = self.create_publisher(Image, "/yolo_front_view", 10)
        self.side_pub = self.create_publisher(Image, "/yolo_side_view", 10)


    def front_callback(self, msg):
        image = self.process_image(msg)
        if image is not None:
            self.front_pub.publish(image)

    def side_callback(self, msg):
        image = self.process_image(msg)
        if image is not None:
            self.side_pub.publish(image)

    def process_image(self, image):

        results = self.model(image, imgsz=224)

        if len(results) == 0:
            return None

        best_cube = None
        best_cube_conf = 0
        best_red = None
        best_red_conf = 0
        best_yellow = None
        best_yellow_conf = 0
        best_blue = None
        best_blue_conf = 0

        for result in results:
            for box in result.boxes:

                array = box.data.cpu().numpy()
                x1, y1, x2, y2, conf, clas = array[0]

                if clas <= 5:
                    if conf > best_cube_conf:
                        best_cube = array
                        best_cube_conf = conf

                elif clas == 6:
                    if conf > best_blue_conf:
                        best_blue = array
                        best_blue_conf = conf

                elif clas == 7:
                    if conf > best_yellow_conf:
                        best_yellow = array
                        best_yellow_conf = conf

                elif clas == 8:
                    if conf > best_red_conf:
                        best_red = array
                        best_red_conf = conf

            if best_cube is not None:
                x1, y1, x2, y2, conf, clas = best_cube[0]
                if clas in [0, 3]:
                    corr_tray = best_blue
                if clas in [1, 4]:
                    corr_tray = best_yellow
                if clas in [2, 5]:
                    corr_tray = best_red

            trays = []
            if best_blue is not None:
                trays.append(best_blue[0])
            if best_yellow is not None:
                trays.append(best_yellow[0])
            if best_red is not None:
                trays.append(best_red[0])


            # Sort from left to right
            trays.sort(key=lambda x: x[0])

            for rect in [best_cube, corr_tray]:
                if rect is None:
                    continue

                x1, y1, x2, y2, conf, clas = rect[0]

                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                clas = int(clas)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(image, classes[clas], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
