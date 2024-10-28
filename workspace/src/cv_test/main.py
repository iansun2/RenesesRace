import cv2
import numpy as np

cap = cv2.VideoCapture(0)
assert cap.isOpened(), f'Failed to open'
img = np.random.rand(200,300)
cv2.imshow("test", img)
cv2.waitKey(10000)