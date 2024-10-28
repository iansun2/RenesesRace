import cv2
import numpy as np

print(f"CV2 Version {cv2.__version__}")
print(f"CV2 Path {cv2.__file__}")

cap = cv2.VideoCapture(0)
assert cap.isOpened(), f'Failed to open'
print("open finish")

while 1:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("img", frame)
    cv2.waitKey(1)