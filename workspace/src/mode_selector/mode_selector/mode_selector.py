import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np
import cv2
import redis
import time
import json

class ModeSelectorNode(Node):
    def __init__(self):
        super().__init__('mode_selector_node')
        self.get_logger().info("init start")
        ''' data stream '''
        self.rds = redis.Redis(host='localhost', port=6379, db=0)
        self.pub_trace_cfg = self.create_publisher(String, '/trace_cfg', 1)
        self.pub_avoidance_cfg = self.create_publisher(String, '/avoidance_cfg', 1)
        self.pub_motor_dist = self.create_publisher(Twist, '/motor/dist', 1)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.on_receive_scan, 1)
        ''' timer ''' 
        self.redis_timer = self.create_timer(0.03, self.redis_timer_callback)
        ''' parameter '''

        self.get_logger().info("init end")


    def redis_timer_callback(self):
        ''' try receive yolo result '''
        yolo_result = self.rds.get('yolo_detect')
        if yolo_result:
            yolo_result = json.loads(yolo_result)
            
        


def main(args=None):
    rclpy.init(args=args)
    node = ModeSelectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()