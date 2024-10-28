import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraRGBNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/rgb', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.frame_id = 0
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            cv2.imshow("camera", img)
            cv2.waitKey(1)
            # self.get_logger().info('new image')
            msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            msg.header.frame_id = str(self.frame_id)
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Error reading from video device')

def main(args=None):
    rclpy.init(args=args)
    usb_cam_rgb_node = USBCameraRGBNode()
    rclpy.spin(usb_cam_rgb_node)
    usb_cam_rgb_node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()