import matplotlib.pylab as plt
import numpy as np
import cv2

image = cv2.imread("\lane.jpg")

# converting the image into rgb format
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

plt.imshow(image)
plt.show()
