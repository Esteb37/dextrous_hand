import os
import cv2
import numpy as np

classes = ["big_blue", "big_yellow", "big_red", "small_blue", "small_yellow", "small_red", "tray_blue", "tray_yellow", "tray_red"]

colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Get the current path
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Get the current folder name only
class_name = os.path.basename(os.getcwd())

print("class", class_name)

class_index = classes.index(class_name)


current_path = os.getcwd()

top_right = None
bottom_left = None
temp_bottom_left = None

def drag_event(event, x, y, flags, param):
    global top_right, bottom_left, temp_bottom_left
    if event == cv2.EVENT_LBUTTONDOWN:
        top_right = (x, y)
    # while draggin
    elif event == cv2.EVENT_MOUSEMOVE and top_right is not None:
        temp_bottom_left = (x, y)
    # when draggin stops
    elif event == cv2.EVENT_LBUTTONUP:
        bottom_left = (x, y)

cv2.namedWindow("image")
cv2.setMouseCallback("image", drag_event)

files = os.listdir(current_path)

# Sort by name
files.sort()

for file in files:
    name = current_path + "/" + file

    if os.path.isfile(current_path +"/"+ file[:-4] + ".txt"):
        continue

    img = cv2.imread(name)
    if img is not None:

        coordinates = []
        current_class = class_index

        print(classes[current_class])
        while top_right is None or bottom_left is None:
            copy = img.copy()
            if (top_right is not None and temp_bottom_left is not None):
                    cv2.rectangle(copy, top_right, temp_bottom_left, colors[current_class], 2)
                    cv2.putText(copy, classes[current_class], (top_right[0], top_right[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colors[current_class], 2)

            cv2.imshow("image", copy)
            cv2.waitKey(1)

        center_x = int((top_right[0] + bottom_left[0]) / 2) / 224
        center_y = int((top_right[1] + bottom_left[1]) / 2) / 224
        width = abs(top_right[0] - bottom_left[0]) / 224
        height = abs(top_right[1] - bottom_left[1]) / 224
        top_right = None
        bottom_left = None
        temp_bottom_left = None
        coordinates.append(str(current_class) + " " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height))
        img = copy.copy()

        with open(current_path +"/"+ file[:-4] + ".txt", "w") as f:
                for coordinate in coordinates:
                        f.write(coordinate + "\n")
        print("File " + file + " done")