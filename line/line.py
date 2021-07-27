import cv2
import numpy as np
import math
import frame



#loop through frames for now, no framerate set yet
while True:
	capture = cv2.VideoCapture('lines.mkv')
	ret, single_frame = capture.read()
	frame.frame_analyze(single_frame)
	cv2.waitKey(0)
