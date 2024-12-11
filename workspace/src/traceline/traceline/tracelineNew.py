import cv2
import numpy as np
import copy, json, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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
        self.get_logger().info('init start')
        self.sub_img = self.create_subscription(
            Image,
            '/rgb',
            self.receive_image_callback,
            1)
        self.sub_cfg = self.create_subscription(
            String,
            '/trace_cfg',
            self.on_receive_cfg,
            5)
        self.bridge = CvBridge()
        self.pub_mot = self.create_publisher(Twist, '/motor/main', 10)
        '''  Config '''
        self.enable = True
        self.mode = 'dual'
        self.mix_TMB = [0.5, 0.3, 0.2]
        self.pid = [5, 0, 0]
        self.speed = 0.06
        self.ref = 0.33
        self.get_logger().info('init finish')
        ''' Variable '''
        self.fps_counter = 0
        self.last_count_time = time.time()


    def on_receive_cfg(self, msg: String):
        config = json.loads(msg.data)
        self.get_logger().info(f"receive config:\n{config}")
        if config.get('en') is not None:
            self.enable = config['en']
        if config.get('mode') is not None:
            self.mode = config['mode']
        if config.get('mix_TMB') is not None:
            self.mix_TMB = config['mix_TMB']
        if config.get('pid') is not None:
            self.pid = config['pid']
        if config.get('speed') is not None:
            self.speed = config['speed']
        if config.get('ref') is not None:
            self.ref = config['ref']


    def receive_image_callback(self, msg):
        if not self.enable:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imshow("receive", img)
        trace, debug_img = self.get_trace_value(img)
        self.get_logger().info(f"trace: {trace}")
        msg = Twist()
        msg.linear.z = float(self.speed)
        msg.angular.z = float(trace)
        self.pub_mot.publish(msg)
        cv2.imshow("traceline", debug_img)
        cv2.waitKey(1)
        ''' fps '''
        self.fps_counter += 1
        if time.time() - self.last_count_time >= 2:
            self.get_logger().info(f"fps: {self.fps_counter / 2}")
            self.fps_counter = 0
            self.last_count_time = time.time()


    def get_trace_value(self, frame : cv2.UMat) -> tuple[float, cv2.UMat]:
        rsize = frame.shape
        frame = frame[200:rsize[0]-100, 100:rsize[1]-100, :]
        frame = cv2.resize(frame, (320, 200))
        fsize = frame.shape
        ''' preprocess ''' 
        gau_frame = cv2.GaussianBlur(frame, (9,9), 0)
        hsv = cv2.cvtColor(gau_frame, cv2.COLOR_BGR2HSV)
        # mask_R = cv2.inRange(hsv,lower_R,upper_R)
        # mask_L = cv2.inRange(hsv,lower_L,upper_L)
        hsv_h = hsv[:, :, 2]
        # gau_hsv_h = cv2.GaussianBlur(hsv_h, (9,9), 0)
        canny = cv2.Canny(hsv_h, 80, 120)
        kernel = np.ones((9,9), np.uint8)
        canny = cv2.dilate(canny, kernel)
        ## debug frame
        # debug_frame = copy.deepcopy(canny)
        debug_frame = copy.deepcopy(frame)
        ''' find trace '''
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
        
        ''' calculate sum in 3 section '''
        avg_L = [0] * 3
        avg_L_cnt = [0] * 3
        avg_R = [0] * 3
        avg_R_cnt = [0] * 3
        section = [150, 100, 50] # seg0(bottom): >150, seg1(mid): 150-100, seg2(top): 100-50
        ## left
        for p in road_edge_point_L:
            cv2.circle(debug_frame, p, 2, (255, 0, 0), 2)
            if p[1] > section[0]:
                avg_L[0] += p[0]
                avg_L_cnt[0] += 1
            elif p[1] > section[1]:
                avg_L[1] += p[0]
                avg_L_cnt[1] += 1
            elif p[1] > section[2]:
                avg_L[2] += p[0]
                avg_L_cnt[2] += 1
        ## sum to avg
        avg_L[0] /= avg_L_cnt[0] if avg_L_cnt[0] else 1
        avg_L[1] /= avg_L_cnt[1] if avg_L_cnt[1] else 1
        avg_L[2] /= avg_L_cnt[2] if avg_L_cnt[2] else 1
        # print(avg_L)
        ## right
        for p in road_edge_point_R:
            cv2.circle(debug_frame, p, 2, (0, 255, 0), 2)
            if p[1] > section[0]:
                avg_R[0] += p[0]
                avg_R_cnt[0] += 1
            elif p[1] > section[1]:
                avg_R[1] += p[0]
                avg_R_cnt[1] += 1
            elif p[1] > section[2]:
                avg_R[2] += p[0]
                avg_R_cnt[2] += 1
        ## when no point, set section avg to img width
        avg_R[0] = avg_R[0] if avg_R[0] else fsize[1]
        avg_R[1] = avg_R[1] if avg_R[1] else fsize[1]
        avg_R[2] = avg_R[2] if avg_R[2] else fsize[1]
        ## sum to avg
        avg_R[0] /= avg_R_cnt[0] if avg_R_cnt[0] else 1
        avg_R[1] /= avg_R_cnt[1] if avg_R_cnt[1] else 1
        avg_R[2] /= avg_R_cnt[2] if avg_R_cnt[2] else 1
        # print(avg_R)
        ''' output '''
        avg = []
        if self.mode == 'dual':
            ## combine LR
            for idx in range(len(avg_L)):
                avg.append(int((avg_L[idx] + avg_R[idx]) / 2))
        elif self.mode == 'left':
            avg = avg_L
        else:
            avg = avg_R
        avg = np.array(avg)
        cv2.circle(debug_frame, (avg[0], section[0] + 25), 2, (255, 255, 255), 2)
        cv2.circle(debug_frame, (avg[1], section[1] + 25), 2, (255, 255, 255), 2)
        cv2.circle(debug_frame, (avg[2], section[2] + 25), 2, (255, 255, 255), 2)
        output_seg = h_middle - avg
        output = output_seg[0] * self.mix_TMB[2] \
            + output_seg[1] * self.mix_TMB[1] \
            + output_seg[2] * self.mix_TMB[0]
        return output, debug_frame



def main(args=None):
    rclpy.init(args=args)
    traceline_node = TraceLineNode()
    rclpy.spin(traceline_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()