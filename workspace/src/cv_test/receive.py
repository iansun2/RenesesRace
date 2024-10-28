import cv2
import redis
import numpy as np
import time

rds = redis.Redis(host='localhost', port=6379, db=0)
frame_cnt = 0
last_print = time.time()

while True:
    try:
        img_buff = rds.get('camera')
        # img = np.frombuffer(img_buff, np.uint8).reshape(720, 1280, 3)
        img = np.frombuffer(img_buff, np.uint8).reshape(480, 640, 3)
        frame_cnt += 1
        if time.time() - last_print >= 1:
            last_print = time.time()
            print(f"{frame_cnt} fps, {1000 / frame_cnt} ms")
            frame_cnt = 0
        # cv2.imshow("receive", img)
        cv2.waitKey(1)
    except Exception as e:
        print("error: ", e)