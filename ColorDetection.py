import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 150, 175]) #3, 215, 180 // 0, 100, 175 // 3, 226, 226 // 0, 209, 209
    upper_orange = np.array([40, 255, 255]) #255, 255, 255 // 40, 255, 255

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('frame', result)
    if cv2.waitKey(1) == ord('q'):
        break

print('size', result.size)
print('final shape:', result.shape)

cap.release()
cv2.destroyAllWindows()

#  BGR_color = np.array([[[31, 103, 255]]])
