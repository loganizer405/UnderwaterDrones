import cv2
import numpy as np

#GAUSSIAN
KERNEL_SIZE = 5

#CANNY
LOW_THRESHOLD = 50
HIGH_THRESHOLD = 150

def frame_analyze(frame):
    #I FILTER COLOR

    #change frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #smooth image with a gaussian blur
    '''
    in general computes a weighted average of each pixel and neighbors
    used to reduce noise and smooth the image allowing an image to be defined by its prominent
    features

    The standard deviation is used in the discrete gaussian function that is bounded by the 
    kernel size. The matrix of the approximation is then convolved with each pixel to generate 
    a blurred image.

    explaination: https://homepages.inf.ed.ac.uk/rbf/HIPR2/gsmooth.htm
    ''' 

    # parameter 1: image source parameter 
    # parameter 2: kernel dimmensions as a 2d size 
    # parameter 3: kernel standard deviation in the x direction
    blur_frame = cv.GaussianBlur(gray_frame, (KERNEL_SIZE,KERNEL_SIZE), 0)

    #II DETECT EDGES
   
    '''
    To have an image composed soley of edges will use the Canny edge detection method.
    Multistepped process

    1: gaussian filter is applied
       
        already been used so this can be left out  
        used again

    2: Intensity gradients are found using a Sobel kernel, 2 convolutional 3x3 kernels one finding horizontal gradients and the other find the vertical ones.
    '''
    edged_frame = cv2.Canny(blur_frame, LOW_THRESHOLD, HIGH_THRESHOLD)


    #III INTERPRET THE LINE


    #IV DETERMINE MESSAGE TO SEND TO MAVLINK
    




