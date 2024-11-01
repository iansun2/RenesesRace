import cv2
import numpy as np
import copy
import pandas as pd
from scipy import stats
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


L_H_low = 20
L_H_high = 40
L_S_low = 20
L_S_high = 200
L_V_low = 60
L_V_high = 255

lower_L = np.array([L_H_low,L_S_low,L_V_low])
upper_L = np.array([L_H_high,L_S_high,L_V_high])

R_H_low = 0
R_H_high = 180
R_S_low = 0
R_S_high = 50
R_V_low = 100
R_V_high = 255

lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])



class TraceLineNode(Node):
    def __init__(self):
        super().__init__('traceline_new_node')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.receive_image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/motor', 10)

    def receive_image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imshow("receive", img)
        output_sum, output, debug_img = self.get_trace_value(img)
        self.get_logger().info(f"sum: {output_sum}, section: {output}")
        msg = Twist()
        msg.linear.z = 70.0 / 10.0
        msg.angular.z = output_sum / 2.0
        self.publisher_.publish(msg)
        # cv2.imshow("traceline", debug_img)
        cv2.waitKey(1)


    def get_trace_value(self, frame : cv2.UMat):
        rsize = frame.shape
        frame = frame[200:rsize[0]-100, 100:rsize[1]-100, :]
        frame = cv2.resize(frame, (320, 200))
        
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

        section = [150, 100, 50]
        
        for p in road_edge_point_L:
            # cv2.circle(debug_frame, p, 2, (255, 0, 0), 2)
            if p[1] > section[0]:
                avg_L[0] += p[0]
                avg_L_cnt[0] += 1
            elif p[1] > section[1]:
                avg_L[1] += p[0]
                avg_L_cnt[1] += 1
            elif p[1] > section[2]:
                avg_L[2] += p[0]
                avg_L_cnt[2] += 1
        
        avg_L[0] /= avg_L_cnt[0] if avg_L_cnt[0] else 1
        avg_L[1] /= avg_L_cnt[1] if avg_L_cnt[1] else 1
        avg_L[2] /= avg_L_cnt[2] if avg_L_cnt[2] else 1
        # print(avg_L)
        
        for p in road_edge_point_R:
            # cv2.circle(debug_frame, p, 2, (0, 255, 0), 2)
            if p[1] > section[0]:
                avg_R[0] += p[0]
                avg_R_cnt[0] += 1
            elif p[1] > section[1]:
                avg_R[1] += p[0]
                avg_R_cnt[1] += 1
            elif p[1] > section[2]:
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
        
        # cv2.circle(debug_frame, (avg[0], section[0] + 25), 2, (255, 255, 255), 2)
        # cv2.circle(debug_frame, (avg[1], section[1] + 25), 2, (255, 255, 255), 2)
        # cv2.circle(debug_frame, (avg[2], section[2] + 25), 2, (255, 255, 255), 2)

        output = h_middle - (np.array(avg_L) + np.array(avg_R)) / 2
        output_sum = output[0] * 0.2 + output[1] * 0.4 + output[2] * 0.8
        return output_sum, output, debug_frame



def main(args=None):
    rclpy.init(args=args)
    traceline_node = TraceLineNode()
    rclpy.spin(traceline_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()