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
        self.pub_motor_pos = self.create_publisher(Twist, '/motor/pos', 1)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.on_receive_scan, 1)
        ''' timer ''' 
        self.redis_timer = self.create_timer(0.03, self.redis_timer_callback)
        ''' parameter '''
        ''' variable '''
        self.yolo_filter = []
        self.last_yolo_result_idx = -1
        self.current_mode = 'init'
        self.get_logger().info("init end")


    def redis_timer_callback(self):
        new_result = False
        ''' try receive yolo result '''
        yolo_result = self.rds.get('yolo_detect')
        yolo_result_idx = self.rds.get('yolo_detect_idx')
        # not None check
        if yolo_result is not None and yolo_result_idx is not None:
            # new result
            if yolo_result_idx != self.last_yolo_result_idx:
                new_result = True
            # parse result
            if new_result:
                yolo_result = json.loads(yolo_result)
                yolo_result = yolo_result['detect']
                for result in yolo_result:
                    prob = result['prob']
                    label = result['label']
                    center_x = result['center_x']
                    center_y = result['center_y']
                    box_w = result['box_w']
                    box_h = result['box_h']
                    # first filter
                    if prob > 80 and box_w > 100 and box_h > 100:
                        self.yolo_filter.append(label)
            # end when no new result
            else:
                return
        ''' filter '''
        filter_set = set(self.yolo_filter)
        for label in filter_set:
            # yolo filter label count enougth
            if self.yolo_filter.count(label) > 3:
                self.change_mode(label)
                self.yolo_filter = []
        
                
    def change_mode(self, mode: str):
        ''' template '''
        TRACE_CFG = {
            'en': True,
            'mode': 'dual',
            'mix_TMB'   : [0.5, 0.3, 0.2],
            'ref_LR'    : [0.33, 0.66],
            'pid'       : [5, 0, 0],
            'speed'     : 60
        }
        AVOIDANCE_CFG = {
            'en': True,
            'fov'           : 180,
            'weight_min'    : 0.1,
            'distance'      : 0.4,
            'distance_kp'   : 0.1,
        }
        # exit when mode not change
        if self.current_mode == mode:
            return
        ''' mode change '''
        self.current_mode = mode
        if mode == 'trace':
            ## trace cfg
            trace_cfg = {
                'en': True,
                'mode': 'dual',
                'mix_TMB'   : [0.5, 0.3, 0.2],
                'ref_LR'    : [0.33, 0.66],
                'pid'       : [5, 0, 0],
                'speed'     : 60
            }
            msg = String
            msg.data = json.dumps(trace_cfg)
            self.pub_trace_cfg.publish(msg)
            ## avoidance cfg
            avoidance_cfg = {
                'en': False,
            }
            msg = String
            msg.data = json.dumps(avoidance_cfg)
            self.pub_avoidance_cfg.publish(msg)
        elif mode == 'avoidance':
            ## trace cfg
            trace_cfg = {
                'en': True,
                'mode': 'dual',
                'mix_TMB'   : [0.5, 0.3, 0.2],
                'ref_LR'    : [0.33, 0.66],
                'pid'       : [5, 0, 0],
                'speed'     : 60
            }
            msg = String
            msg.data = json.dumps(trace_cfg)
            self.pub_trace_cfg.publish(msg)
            ## avoidance cfg
            avoidance_cfg = {
                'en': True,
                'fov'           : 180,
                'weight_min'    : 0.1,
                'distance'      : 0.4,
                'distance_kp'   : 0.1,
            }
            msg = String
            msg.data = json.dumps(avoidance_cfg)
            self.pub_avoidance_cfg.publish(msg)
        elif mode == 'parking':
            ## trace cfg
            trace_cfg = {
                'en': False,
            }
            msg = String
            msg.data = json.dumps(trace_cfg)
            self.pub_trace_cfg.publish(msg)
            ## avoidance cfg
            avoidance_cfg = {
                'en': False,
            }
            msg = String
            msg.data = json.dumps(avoidance_cfg)
            self.pub_avoidance_cfg.publish(msg)
            pass


    def on_receive_scan(self, msg: LaserScan):
        
        pass
                
                
            
        


def main(args=None):
    rclpy.init(args=args)
    node = ModeSelectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()