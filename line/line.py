import cv2
import numpy as np
import math
import frame

#open camera
def camera_open(num):
    cap = cv.VideoCapture(num)
    if not cap.isOpened():
        print("cannot open camera")
        exit()
    return cap


capture = camera_open(0)


#loop through frames for now, no framerate set yet
while True:
    ret, frame = capture.read()
    message = frame_analyze(frame)
    if not ret:
        print("can't recieve frame")
        exit()
    

