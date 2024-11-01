import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import redis
import time

class USBCameraRGBNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("init start")
        # data stream
        self.publisher_ = self.create_publisher(Image, '/rgb', 10)
        self.rds = redis.Redis(host='localhost', port=6379, db=0)
        self.pipe = self.rds.pipeline()
        # rclpy 
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()
        self.declare_parameter('camera', 0)
        self.declare_parameter('file', '')
        camera = self.get_parameter('camera').get_parameter_value().integer_value
        file = self.get_parameter('file').get_parameter_value().string_value
        if file == '':
            self.src = camera
        else:
            self.src = file
        # fps counter
        self.fps = 0
        self.last_count_fps = time.time()
        # cap
        self.frame_id = 0
        self.get_logger().info("init cap")
        self.get_logger().info(f"open cap: {self.src}")
        self.cap = cv2.VideoCapture()
        self.cap.open(self.src, apiPreference=cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # auto expos
        time.sleep(3)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual expos
        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 80)
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # self.cap.set(cv2.CAP_PROP_SATURATION, 0)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video source {self.src}")
            rclpy.shutdown()
        self.get_logger().info("init end")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.fps += 1
            if time.time() - self.last_count_fps >= 1:
                self.last_count_fps = time.time()
                self.get_logger().info(f"{self.fps} fps, {1000 / self.fps :.2f} ms")
                self.fps = 0
            # frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            # cv2.imshow("camera", frame)
            cv2.waitKey(1)
            # self.get_logger().info('new image')
            self.rds.set('camera', frame.tostring())
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.frame_id = str(self.frame_id)
            self.pipe.execute()
            self.publisher_.publish(msg)
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