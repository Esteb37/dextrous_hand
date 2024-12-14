#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

bridge = CvBridge()

classes = ["big_blue", "big_yellow", "big_red", "small_blue", "small_yellow", "small_red", "tray_blue", "tray_yellow", "tray_red"]

class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.model = YOLO("/home/esteb37/ros2_ws/src/dextrous_hand/dextrous_hand/yolo/runs/detect/train7/weights/best.pt")

        self.front_sub = self.create_subscription(Image, "/oakd_front_view/color", self.front_callback, 10)
        self.side_sub = self.create_subscription(Image, "/oakd_side_view/color", self.side_callback, 10)

        self.front_pub = self.create_publisher(Image, "/yolo_front_view", 10)
        self.side_pub = self.create_publisher(Image, "/yolo_side_view", 10)
        self.decision_pub = self.create_publisher(String, "/yolo_decision", 10)

        self.front_decision = "unsure"
        self.side_decision = "unsure"

        self.decide_looper = self.create_timer(0.1, self.decide_loop)

        self.side_init = False
        self.front_init = False

        self.start_time = float(self.get_clock().now().nanoseconds)
        self.decision = "unsure"

        self.get_logger().warn("Yolo node started")


    def front_callback(self, msg):
        self.front_init = True
        if self.decision != "unsure":
            return

        side, image = self.process_image(msg, "front")
        self.front_decision = side
        if image is not None:
            image = bridge.cv2_to_imgmsg(image, "bgr8")
            self.front_pub.publish(image)

    def side_callback(self, msg):
        self.side_init = True
        if self.decision != "unsure":
            return

        side, image = self.process_image(msg, "side")
        self.side_decision = side
        if image is not None:
            image = bridge.cv2_to_imgmsg(image, "bgr8")
            self.side_pub.publish(image)

    def decide(self):
        if self.front_decision != self.side_decision:
            return "unsure"

        return self.front_decision

    def decide_loop(self):
        if self.decision != "unsure":
            msg = String()
            msg.data = self.decision
            self.decision_pub.publish(msg)
            return

        if not self.front_init or not self.side_init:
            self.start_time = float(self.get_clock().now().nanoseconds)
            return

        stop_time = float(self.get_clock().now().nanoseconds)

        if stop_time - self.start_time > 2e9:
            self.get_logger().warn("YOLO unsure")
            msg = String()
            msg.data = "unsure"
            self.decision_pub.publish(msg)

        self.decision = self.decide()

        if self.decision == "unsure":
            self.get_logger().info("YOLO unsure for: " + str((stop_time - self.start_time)/1e9) + " seconds")

        else:
            msg = String()
            msg.data = self.decision
            self.decision_pub.publish(msg)

    def process_image(self, image, camera, visualize = False):

        image = bridge.imgmsg_to_cv2(image)

        results = self.model(image, imgsz=224)

        best_cube = None
        best_cube_conf = 0
        best_red = None
        best_red_conf = 0
        best_yellow = None
        best_yellow_conf = 0
        best_blue = None
        best_blue_conf = 0

        side = "unsure"

        for result in results:
            for box in result.boxes:

                array = box.data.cpu().numpy()[0]
                x1, y1, x2, y2, conf, clas = array

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
                x1, y1, x2, y2, conf, clas = best_cube
                if clas in [0, 3] and best_blue is not None:
                    corr_tray = best_blue
                if clas in [1, 4] and best_yellow is not None:
                    corr_tray = best_yellow
                if clas in [2, 5] and best_red is not None:
                    corr_tray = best_red

                if corr_tray is None:
                    return

                trays = []
                if best_blue is not None:
                    trays.append(best_blue)
                if best_yellow is not None:
                    trays.append(best_yellow)
                if best_red is not None:
                    trays.append(best_red)

                if camera == "front":
                    trays.sort(key=lambda x: x[0])
                else:
                    trays.sort(key=lambda x: x[0], reverse=True)

                side = "left"

                if trays[2][5] == corr_tray[5]:
                    side = "right"

                if trays[1][5] == corr_tray[5]:
                    side = "center"

                if visualize:
                    for rect in [best_cube, corr_tray]:
                        if rect is None:
                            continue

                        x1, y1, x2, y2, conf, clas = rect

                        x1 = int(x1)
                        y1 = int(y1)
                        x2 = int(x2)
                        y2 = int(y2)
                        clas = int(clas)
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        cv2.putText(image, classes[clas], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        if visualize:
            cv2.putText(image, side, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            return side, image

        else:
            return side, None


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
