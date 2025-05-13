import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import redis
import time

class USBCameraRGBNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("init start")
        # data stream
        self.publisher_ = self.create_publisher(Image, '/rgb', 1)
        self.rds = redis.Redis(host='localhost', port=6379, db=0)
        self.pipe = self.rds.pipeline()
        # rclpy 
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        # parameter
        self.declare_parameter('src', '')
        src_str = self.get_parameter('src').get_parameter_value().string_value
        # use camera 0
        if src_str == "":
            self.src = 0 + cv2.CAP_V4L2
        # is camera (number)
        elif src_str.isnumeric():
            self.src = int(src_str) + cv2.CAP_V4L2
        # is filename (other)
        else:
            self.src = src_str
        # fps counter
        self.fps = 0
        self.last_count_fps = time.time()
        # cap
        self.frame_id = 0
        self.get_logger().info("init cap")
        self.get_logger().info(f"open cap: {self.src}")
        self.cap = cv2.VideoCapture(self.src)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video source {self.src}")
            rclpy.shutdown()
        # config cap when it is camera
        if isinstance(self.src, int):
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # auto expos
            time.sleep(3)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual expos
            # self.cap.set(cv2.CAP_PROP_EXPOSURE, 40)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 80) # C170
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            # self.cap.set(cv2.CAP_PROP_FPS, 10)
        self.get_logger().info("init end")

    def timer_callback(self):
        dt = [[0, time.time()]]
        ret, frame = self.cap.read()
        dt.append([dt[-1][0] + 1, time.time()])
        if ret:
            # print("shape: ", frame.shape)
            self.fps += 1
            if time.time() - self.last_count_fps >= 1:
                self.last_count_fps = time.time()
                self.get_logger().info(f"{self.fps} fps, {1000 / self.fps :.2f} ms")
                self.fps = 0
            # dt.append([dt[-1][0] + 1, time.time()])
            ''' ros2 msg '''
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.frame_id = str(self.frame_id)
            self.publisher_.publish(msg)
            # dt.append([dt[-1][0] + 1, time.time()])
            '''  to square '''
            pad = np.zeros((int((frame.shape[1] - frame.shape[0]) / 2), frame.shape[1], 3), dtype=np.uint8)
            img = np.concatenate((pad, frame, pad), axis=0)
            # dt.append([dt[-1][0] + 1, time.time()])
            # cv2.imshow("img", img)
            ''' redis '''
            self.pipe.set('img', img.tostring())
            result, img_jpg = cv2.imencode(".jpg", img)
            self.pipe.set('img_jpg', img_jpg.tostring())
            self.pipe.set('img_idx', self.frame_id)
            self.pipe.set('img_shape', str(img.shape[0]) + " " + str(img.shape[1]))
            self.pipe.execute()
            dt.append([dt[-1][0] + 1, time.time()])
            ''' end of loop '''
            # print dt with downsample
            if self.frame_id % 5 == 0:
                last_dt = dt[0]
                for t in dt:
                    print(f"{int((t[1] - last_dt[1])*1000)}, ", end='')
                    last_dt = t
                print("\n================")
            # update variable
            self.frame_id += 1
            cv2.waitKey(1)
        else:
            self.get_logger().error('Error reading from video device')
            self.cap = cv2.VideoCapture(self.src)

def main(args=None):
    rclpy.init(args=args)
    usb_cam_rgb_node = USBCameraRGBNode()
    rclpy.spin(usb_cam_rgb_node)
    usb_cam_rgb_node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()