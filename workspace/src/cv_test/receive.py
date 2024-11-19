import cv2
import redis
import numpy as np
import time
import json

rds = redis.Redis(host='localhost', port=6379, db=0)
last_img_idx = 0
frame_cnt = 0
last_print = time.time()

img_shape = rds.get('img_shape').split()
img_shape = (int(img_shape[0]), int(img_shape[1]))
print("shape:", img_shape)

while True:
    img_idx = rds.get('img_idx')
    if last_img_idx != img_idx:
        yolo_detect = rds.get('yolo_detect')
        img_buff = rds.get('img')
        img = np.frombuffer(img_buff, np.uint8).reshape(img_shape[0], img_shape[1], 3)
        # process img
        json_object = json.loads(yolo_detect)
        detect_list = json_object['detect']
        for detect_obj in detect_list:
            x = detect_obj['center_x']
            y = detect_obj['center_y']
            w = detect_obj['box_w']
            h = detect_obj['box_h']
            label = detect_obj['label']
            prob = detect_obj['prob']
            # cv2.circle(img, (x, y), 5, (255, 0, 255), 2)
            # cv2.rectangle(img, rec=(x, y, w, h), color=(255, 0, 0), thickness=2, lineType=cv2.LINE_8)


        # fps counter
        frame_cnt += 1
        if time.time() - last_print >= 1:
            last_print = time.time()
            print(f"{frame_cnt} fps, {1000 / frame_cnt} ms")
            frame_cnt = 0
        cv2.imshow("receive", img)
        last_img_idx = img_idx
    cv2.waitKey(1)