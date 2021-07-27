import cv2
import math
import numpy as np

#GAUSSIAN
KERNEL_SIZE = 5

#CANNY
LOW_THRESHOLD = 20
HIGH_THRESHOLD = 100


def find_angle(x1,y1,x2,y2):
     a = x2 - x1
     b = y2 - y1 
     return angle_between_vectors(1,0,a,b)

def angle_between_vectors(x1,y1,x2,y2):
    return math.acos((x1 * x2 + y1 * y2) / ((math.sqrt(math.pow(x1,2) + math.pow(y1,2))) * (math.sqrt(math.pow(x2,2) + math.pow(y2,2)))))




def frame_analyze(frame):
    #I FILTER COLOR

    #change frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #smooth image with a gaussian blur

    # parameter 1: image source parameter 
    # parameter 2: kernel dimmensions as a 2d size 
    # parameter 3: kernel standard deviation in the x direction
    blur_frame = cv2.GaussianBlur(gray_frame, (KERNEL_SIZE,KERNEL_SIZE), 0)

    #II DETECT EDGES
    edged_frame = cv2.Canny(blur_frame, LOW_THRESHOLD, HIGH_THRESHOLD)


    #III INTERPRET THE LINE

    # hough line transform
    rho = 1
    theta = np.pi
    threshold = 15
    min_line_length = 50
    max_line_gap = 20
    line_frame = np.copy(frame)
    lines = cv2.HoughLinesP(edged_frame, rho, theta, threshold, np.array([]),
            min_line_length, max_line_gap)

    
    if len(lines) == 0:
        return
    print("new frame")
    for line in lines:
        for x1, y1, x2, y2 in line:
            angle = find_angle(x1,y1,x2,y2)           
            print(angle * (math.pi/180))

    




