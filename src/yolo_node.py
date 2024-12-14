#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

bridge = CvBridge()

classes = ["big_blue", "big_yellow", "big_red", "small_blue", "small_yellow", "small_red", "tray_blue", "tray_yellow", "tray_red"]

tray_positions = ["left", "center", "right", "unsure"]

class YOLONode(Node):

    ACCEPTANCE_TIMER = 0.3
    DISCREPANCY_TIMER = 5

    def __init__(self):
        super().__init__("yolo_node")
        self.model = YOLO("/home/esteb37/ros2_ws/src/dextrous_hand/dextrous_hand/yolo/runs/detect/train7/weights/best.pt")

        self.front_sub = self.create_subscription(Image, "/oakd_front_view/color", self.front_callback, 10)
        self.side_sub = self.create_subscription(Image, "/oakd_side_view/color", self.side_callback, 10)

        self.front_pub = self.create_publisher(Image, "/yolo_front_view", 10)
        self.side_pub = self.create_publisher(Image, "/yolo_side_view", 10)
        self.tray_pub = self.create_publisher(Int32, "/yolo_decision", 10)
        self.cube_pub = self.create_publisher(String, "/cube_size", 10)

        self.front_tray = "unsure"
        self.side_tray = "unsure"

        self.front_boxes = [None, None, None, None]
        self.side_boxes = [None, None, None, None]

        self.cube_looper = self.create_timer(0.1, self.cube_loop)

        self.side_init = False
        self.front_init = False

        self.cube_start_time = float(self.get_clock().now().nanoseconds)
        self.tray_start_time = float(self.get_clock().now().nanoseconds)

        self.tray_position = "initial"
        self.cube_size = "initial"
        self.last_cube_size = "initial"
        self.last_tray_position = "initial"

        self.cube_uncertainty_start_time = None
        self.tray_uncertainty_start_time = None

        self.visualize = True

        self.get_logger().warn("Yolo node started")

    def front_callback(self, msg):
        self.front_init = True

        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        boxes = self.get_boxes(image)

        self.front_boxes = boxes

        if self.visualize:
            self.draw(image, boxes, "front")
            image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
            self.front_pub.publish(image_msg)

    def side_callback(self, msg):
        self.side_init = True

        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        image = cv2.rotate(image, cv2.ROTATE_180)

        boxes = self.get_boxes(image)

        self.side_boxes = boxes

        if self.visualize:
            self.draw(image, boxes, "side")
            image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
            self.side_pub.publish(image_msg)

    def decide(self, front, side):
        if front != side:
            if front == "unsure":
                return side
            if side == "unsure":
                return front
            else:
                return "discrepancy"

        return front

    def cube_loop(self):
        if self.front_init and self.side_init:
            front_cube, front_blue, front_yellow, front_red = self.front_boxes
            side_cube, side_blue, side_yellow, side_red = self.side_boxes

            update_tray = True

            if front_cube is None:
                front_size = self.cube_size
            else:
                _, y1, _, y2, _, clas = front_cube

                ymin =  int(224 * 0.6)

                if (y1 + y2) / 2 > ymin:
                    front_size = "small" if clas in [3, 4, 5] else "big"

                else:
                    front_size = self.cube_size
                    update_tray = False

            if side_cube is None:
                side_size = self.cube_size
            else:
                _, y1, _, y2, _, clas = side_cube

                ymin =  int(224 * 0.5)

                if (y1 + y2) / 2 > ymin:
                    side_size = "small" if clas in [3, 4, 5] else "big"
                else:
                    side_size = self.cube_size
                    update_tray = False

            cube_size = self.decide(front_size, side_size)

            if cube_size != "discrepancy" and cube_size != "unsure":
                self.cube_uncertainty_start_time = None
                if cube_size != self.last_cube_size:
                    self.cube_start_time = float(self.get_clock().now().nanoseconds)

                self.last_cube_size = cube_size

                if self.get_clock().now().nanoseconds - self.cube_start_time > 1e9 * self.ACCEPTANCE_TIMER:
                    if self.cube_size != cube_size:
                        self.get_logger().warn(f"Cube size: {cube_size}")
                        self.cube_size = cube_size

                if update_tray:
                    front_tray, _ = self.get_tray_side(front_cube, front_blue, front_yellow, front_red, "front")
                    side_tray, _ = self.get_tray_side(side_cube, side_blue, side_yellow, side_red, "side")

                    tray_position = self.decide(front_tray, side_tray)

                    if tray_position == "discrepancy" or tray_position == "unsure":
                        if self.tray_position == "initial":
                            if self.tray_uncertainty_start_time is None:
                                self.tray_uncertainty_start_time = float(self.get_clock().now().nanoseconds)

                            if self.get_clock().now().nanoseconds - self.tray_uncertainty_start_time > 1e9 * self.DISCREPANCY_TIMER:
                                self.tray_uncertainty_start_time = None
                                self.get_logger().error("Unable to get tray position, assuming center")
                                self.tray_position = "center"
                        else:
                            self.last_tray_position = self.tray_position

                    elif tray_position != self.tray_position:
                        if self.last_tray_position != tray_position:
                            self.tray_start_time = float(self.get_clock().now().nanoseconds)

                        if self.get_clock().now().nanoseconds - self.tray_start_time > 1e9 * self.ACCEPTANCE_TIMER:
                            self.tray_position = tray_position
                            self.get_logger().warn(f"Tray position: {self.tray_position}")

                        self.last_tray_position = tray_position


            else:
                if self.cube_uncertainty_start_time is None:
                    self.cube_uncertainty_start_time = float(self.get_clock().now().nanoseconds)

                if self.get_clock().now().nanoseconds - self.cube_uncertainty_start_time > 1e9 * self.DISCREPANCY_TIMER:
                    self.cube_uncertainty_start_time = None
                    self.get_logger().error("Unable to get cube size, assuming big")
                    self.cube_size = "big"

            if self.cube_size != "initial":
                msg = String()
                msg.data = self.cube_size
                self.cube_pub.publish(msg)

            if self.tray_position != "initial":
                msg = Int32()
                msg.data = tray_positions.index(self.tray_position)
                self.tray_pub.publish(msg)


    def get_boxes(self, image):
        results = self.model(image, imgsz=224, verbose=False)

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

                array = box.data.cpu().numpy()[0]
                _, _, _, _, conf, clas = array

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

        return [best_cube, best_blue, best_yellow, best_red]

    def draw(self, image, boxes, camera):

        best_cube, best_blue, best_yellow, best_red = boxes

        if best_cube is not None:
            side, corr_tray = self.get_tray_side(best_cube, best_blue, best_yellow, best_red, camera)

            for rect in [best_cube, corr_tray]:
                if rect is None:
                    continue

                x1, y1, x2, y2, _, clas = rect

                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                clas = int(clas)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(image, classes[clas], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.putText(image, side, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def get_tray_side(self, cube, blue, yellow, red, camera):

        if cube is None:
            return "unsure", None

        x1, _, x2, _, _, clas = cube
        corr_tray = None

        if clas in [0, 3] and blue is not None:
            corr_tray = blue
        if clas in [1, 4] and yellow is not None:
            corr_tray = yellow
        if clas in [2, 5] and red is not None:
            corr_tray = red

        if corr_tray is None:
            side = "unsure"

        else:
            trays = []
            if blue is not None:
                trays.append(blue)
            if yellow is not None:
                trays.append(yellow)
            if red is not None:
                trays.append(red)

            if len(trays) != 3:
                if "side" in camera:
                    center_range = [224 * 0.3, 224 * 0.6]
                else:
                    center_range = [224 * 0.45, 224 * 0.75]

                x_center = (x1 + x2) / 2

                if x_center < center_range[0]:
                    side = "left"
                elif x_center > center_range[1]:
                    side = "right"
                else:
                    side = "center"

            else:
                trays.sort(key=lambda x: x[0])

                side = "left"

                if trays[2][5] == corr_tray[5]:
                    side = "right"

                if trays[1][5] == corr_tray[5]:
                    side = "center"

        return side, corr_tray

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
