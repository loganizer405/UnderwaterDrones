import numpy as np
import cv2

cap = cv2.VideoCapture(0)

for i in range(1000):
    ret, frame = cap.read()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 80)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 60)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([100, 100, 100])
    upper_orange = np.array([255, 255, 255])

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    result = cv2.bitwise_and(frame, frame, mask=mask)
    print(result[2])
    print(result.shape)

    cv2.imshow("frame", result)

    if cv2.waitKey(1) == ord('q'):
        break

print('size', result.size)
print('final shape:', result.shape)

cap.release()
cv2.destroyAllWindows()

#  BGR_color = np.array([[[31, 103, 255]]])
