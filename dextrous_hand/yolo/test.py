from ultralytics import YOLO
import os
import cv2

model = YOLO("/home/esteb37/ros2_ws/src/dextrous_hand/dextrous_hand/yolo/runs/detect/train7/weights/best.pt")

path = "/home/esteb37/Downloads/images/tray_yellow"

files = os.listdir(path)
files.sort()

classes = ["big_blue", "big_yellow", "big_red", "small_blue", "small_yellow", "small_red", "tray_blue", "tray_yellow", "tray_red"]

for file in files:
    name = path + "/" + file

    if os.path is None:
        continue

    image = cv2.imread(name)

    if image is None:
        continue

    results = model(image, imgsz=224)

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
                side = "unsure"
                break

            trays = []
            if best_blue is not None:
                trays.append(best_blue)
            if best_yellow is not None:
                trays.append(best_yellow)
            if best_red is not None:
                trays.append(best_red)

            if "front" in file:
                trays.sort(key=lambda x: x[0])
            else:
                trays.sort(key=lambda x: x[0], reverse=True)

            side = "left"

            if trays[2][5] == corr_tray[5]:
                side = "right"

            if trays[1][5] == corr_tray[5]:
                side = "center"

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

    cv2.putText(image, side, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imshow("image", image)
    cv2.waitKey(0)