import cv2
import os
import numpy as np


path = "/home/esteb37/Downloads/images/train"

dataset_path = "/home/esteb37/Downloads/images/train/dataset"

images = [f for f in os.listdir(path) if f.endswith(".jpg")]

length = len(images)

np.random.shuffle(images)
train = images[:int(length * 0.8)]
test = images[int(length * 0.8):]

train_path = dataset_path + "/train"
test_path = dataset_path + "/test"

for image in train:
    img = cv2.imread(path + "/" + image)
    cv2.imwrite(train_path + "/images/" + image, img)
    os.system("cp " + path + "/" + image[:-4] + ".txt " + train_path+"/labels/")

for image in test:
    img = cv2.imread(path + "/" + image)
    cv2.imwrite(test_path + "/images/" + image, img)
    os.system("cp " + path + "/" + image[:-4] + ".txt " + test_path+"/labels/")

print("Done!")