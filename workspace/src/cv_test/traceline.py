import cv2
import numpy as np
import copy
import pandas as pd
from scipy import stats
import redis
import time 


L_H_low = 20
L_H_high = 40
L_S_low = 15
L_S_high = 150
L_V_low = 120
L_V_high = 255

lower_L = np.array([L_H_low,L_S_low,L_V_low])
upper_L = np.array([L_H_high,L_S_high,L_V_high])

R_H_low = 0
R_H_high = 180
R_S_low = 0
R_S_high =10
R_V_low = 180
R_V_high = 255

lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])

rds = redis.Redis(host='localhost', port=6379, db=0)
pipe = rds.pipeline()



if __name__ == "__main__":
    # cap = cv2.VideoCapture("demo.mp4")
    # if not cap.isOpened():
    #     print("Cannot open cap")
    #     exit()
    
    while True:
        img_buff = rds.get('camera')
        # frame = np.frombuffer(img_buff, np.uint8).reshape(720, 1280, 3)
        frame = np.frombuffer(img_buff, np.uint8).reshape(480, 640, 3)
        ret = True
        if ret:
            rsize = frame.shape
            frame = frame[300:rsize[0]-50, 200:rsize[1]-200, :]
            frame = cv2.resize(frame, (320, 240))
            
            fsize = frame.shape
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # mask_R = cv2.inRange(hsv,lower_R,upper_R)
            # mask_L = cv2.inRange(hsv,lower_L,upper_L)
            
            hsv_h = hsv[:, :, 2]
            gau_hsv_h = cv2.GaussianBlur(hsv_h, (9,9), 0)
            canny = cv2.Canny(gau_hsv_h, 80, 120)
            kernel = np.ones((9,9), np.uint8)
            canny = cv2.dilate(canny, kernel)
            
            # debug_frame = copy.deepcopy(canny)
            debug_frame = copy.deepcopy(frame)
            
            road_edge_point_L = []
            road_edge_point_R = []
            # find 
            h_middle = int(fsize[1] / 2)
            h_offset_max = int(h_middle)
            # print(f"hm {h_middle}, hom {h_offset_max}")
            for v in range(0, fsize[0], 20):
                # find L
                for h in range(h_middle + 100, h_middle - h_offset_max, -5):
                    pixel = np.array([[hsv[v, h - 3, :]]])
                    if canny[v][h] and cv2.inRange(pixel, lower_L, upper_L):
                        target_point = [h, v]
                        road_edge_point_L.append(target_point)
                        break
                # find R
                for h in range(h_middle - 100, h_middle + h_offset_max, 5):
                    pixel = np.array([[hsv[v, h + 3, :]]])
                    if canny[v][h] and cv2.inRange(pixel, lower_R, upper_R):
                        target_point = [h, v]
                        road_edge_point_R.append(target_point)
                        break
                    
            if len(road_edge_point_L) == 0:
                road_edge_point_L.append([0, 0])
                road_edge_point_L.append([0, (fsize[0] - 1) / 4])
                road_edge_point_L.append([0, (fsize[0] - 1) / 2])
                road_edge_point_L.append([0, (fsize[0] - 1) / 4 * 3])
                road_edge_point_L.append([0, fsize[0] - 1])
                
            if len(road_edge_point_R) == 0:
                road_edge_point_R.append([fsize[1] - 1, 0])
                road_edge_point_R.append([fsize[1] - 1, (fsize[0] - 1) / 4])
                road_edge_point_R.append([fsize[1] - 1, (fsize[0] - 1) / 2])
                road_edge_point_R.append([fsize[1] - 1, (fsize[0] - 1) / 4 * 3])
                road_edge_point_R.append([fsize[1] - 1, fsize[0] - 1])
            
            raw_L = np.array(road_edge_point_L)
            df = pd.DataFrame({'x':raw_L[:, 0], 'y':raw_L[:,1]})
            df['z_score'] = stats.zscore(df['x'])
            df = df.loc[df['z_score'].abs() <= 2]
            road_edge_point_L = df[['x', 'y']].to_numpy()
            
            
            raw_R = np.array(road_edge_point_R)
            df = pd.DataFrame({'x':raw_R[:, 0], 'y':raw_R[:,1]})
            df['z_score'] = stats.zscore(df['x'])
            df = df.loc[df['z_score'].abs() <= 2]
            road_edge_point_R = df[['x', 'y']].to_numpy()
            
            
            avg_L = [0] * 3
            avg_L_cnt = [0] * 3
            avg_R = [0] * 3
            avg_R_cnt = [0] * 3
            
            for p in road_edge_point_L:
                cv2.circle(debug_frame, tuple(p), 2, (255, 0, 0), 2)
                if p[1] > 180:
                    avg_L[0] += p[0]
                    avg_L_cnt[0] += 1
                elif p[1] > 120:
                    avg_L[1] += p[0]
                    avg_L_cnt[1] += 1
                elif p[1] > 60:
                    avg_L[2] += p[0]
                    avg_L_cnt[2] += 1
            
            avg_L[0] /= avg_L_cnt[0] if avg_L_cnt[0] else 1
            avg_L[1] /= avg_L_cnt[1] if avg_L_cnt[1] else 1
            avg_L[2] /= avg_L_cnt[2] if avg_L_cnt[2] else 1
            # print(avg_L)
            
            for p in road_edge_point_R:
                cv2.circle(debug_frame, tuple(p), 2, (0, 255, 0), 2)
                if p[1] > 180:
                    avg_R[0] += p[0]
                    avg_R_cnt[0] += 1
                elif p[1] > 120:
                    avg_R[1] += p[0]
                    avg_R_cnt[1] += 1
                elif p[1] > 60:
                    avg_R[2] += p[0]
                    avg_R_cnt[2] += 1
            
            avg_R[0] = avg_R[0] if avg_R[0] else fsize[1]
            avg_R[1] = avg_R[1] if avg_R[1] else fsize[1]
            avg_R[2] = avg_R[2] if avg_R[2] else fsize[1]
             
            avg_R[0] /= avg_R_cnt[0] if avg_R_cnt[0] else 1
            avg_R[1] /= avg_R_cnt[1] if avg_R_cnt[1] else 1
            avg_R[2] /= avg_R_cnt[2] if avg_R_cnt[2] else 1
            # print(avg_R)
            
            avg = []
            for idx in range(len(avg_L)):
                avg.append(int((avg_L[idx] + avg_R[idx]) / 2))
            
            cv2.circle(debug_frame, (avg[0], 210), 2, (255, 255, 255), 2)
            cv2.circle(debug_frame, (avg[1], 150), 2, (255, 255, 255), 2)
            cv2.circle(debug_frame, (avg[2], 90), 2, (255, 255, 255), 2)
            
            
            # cv2.imshow("frame", frame)
            # cv2.imshow("debug", debug_frame)
            # cv2.imshow("mask_R", mask_R)
            # cv2.imshow("mask_L", mask_L)
            # cv2.imshow("canny", canny)
            cv2.waitKey(1)

            L = avg_L[0] * 0.7 + avg_L[1] * 0.4 + avg_L[2] * 0.1
            R = avg_R[0] * 0.7 + avg_R[1] * 0.4 + avg_R[2] * 0.1

            print(f"{time.time()}, L: {L}, R: {R}")
            
    cv2.destroyAllWindows()
        