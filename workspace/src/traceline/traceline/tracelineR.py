import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


# 右線HSV遮色閥值
R_H_low = 0
R_H_high = 180
R_S_low = 0
R_S_high =35
R_V_low = 180
R_V_high = 255

# 右線遮罩
lower_R = np.array([R_H_low,R_S_low,R_V_low])
upper_R = np.array([R_H_high,R_S_high,R_V_high])


# 採樣間距
W_sampling_1 = 305 #325
W_sampling_2 = 250 #290
W_sampling_3 = 205 #255
W_sampling_4 = 150 #220


class TraceLineNode(Node):
    def __init__(self):
        super().__init__('traceline_r_node')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.receive_image_callback,
            10)
        self.bridge = CvBridge()

    def receive_image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imshow("receive", img)
        R_min, debug_img = self.get_trace_value(img)
        self.get_logger().info(f"R: {R_min}")
        cv2.imshow("traceline R", debug_img)
        cv2.waitKey(1)


    def get_trace_value(self, img : cv2.UMat):
        global fork_flag
        # 右線X值(需重置)
        R_min_300 = 640
        R_min_240 = 640
        R_min_180 = 640
        R_min_140 = 640

        # 重設大小、轉HSV
        #img = cv2.resize(img,(640,360))
        img = img[120:480, :]
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        # 右線遮罩
        mask_R = cv2.inRange(hsv, lower_R, upper_R)

        self.get_logger().info("mask finish R")

        # Canny邊緣運算
        kernel_size = 25
        blur_gray = cv2.GaussianBlur(mask_R, (kernel_size, kernel_size), 0)
        low_threshold = 10
        high_threshold = 20
        canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

        self.get_logger().info("gaussian + canny finish R")

        # 閉運算(解緩Canny斷線問題)
        kernel = np.ones((5,5),np.uint8)
        gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

        self.get_logger().info("MORPH_GRADIENT finish R")

        #霍夫變換
        lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
        
        self.get_logger().info("HoughLinesP finish R")

        # Line calculation
        right_min_x = 340
        if type(lines) == np.ndarray:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                if ((x1+x2)/2)>right_min_x and ((y1+y2)/2)>W_sampling_1:
                    # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                    if ((x1+x2)/2)<R_min_300:
                        R_min_300 = int((x1+x2)/2)
                elif ((x1+x2)/2)>right_min_x and ((y1+y2)/2)>W_sampling_2:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                    if ((x1+x2)/2)<R_min_240:
                        R_min_240 = int((x1+x2)/2)
                elif ((x1+x2)/2)>right_min_x and ((y1+y2)/2)>W_sampling_3:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                    if ((x1+x2)/2)<R_min_180:
                        R_min_180 = int((x1+x2)/2)
                elif ((x1+x2)/2)>right_min_x and ((y1+y2)/2)>W_sampling_4:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                    if ((x1+x2)/2)<R_min_140:
                        R_min_140 = int((x1+x2)/2)
        else:
            pass
        
        self.get_logger().info("line process finish R")

        # Right Sum
        R_min = ((R_min_300+R_min_240+R_min_180+R_min_140) / 4) - 320

        # Debug
        pts = np.array( \
            [[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], \
            [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], \
            np.int32)
        pts = pts.reshape((-1, 1, 2))
        img = cv2.polylines(img, [pts], False,(255,200,0),3)
        for point in pts:
            #print("point: ", point)
            cv2.circle(img, tuple(point[0]), 3, color=(255, 0, 200), thickness=3)

        return R_min, img



def main(args=None):
    rclpy.init(args=args)
    traceline_node = TraceLineNode()
    rclpy.spin(traceline_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()