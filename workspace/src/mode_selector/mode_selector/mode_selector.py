import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from motor_if.srv import MotorPos

import numpy as np
import cv2
import redis
import time
import json
import math as m


class_name = [
    'fork',
    'cross',
    'right',
    'park',
    'left?',
    'avoidance',
    '6',
    '7',
    'stop'
]


class ModeSelectorNode(Node):
    def __init__(self):
        super().__init__('mode_selector_node')
        self.get_logger().info("init start")
        ''' data stream '''
        self.rds = redis.Redis(host='localhost', port=6379, db=0)
        self.pub_trace_cfg = self.create_publisher(String, '/trace_cfg', 5)
        self.pub_avoidance_cfg = self.create_publisher(String, '/avoidance_cfg', 5)
        self.pub_motor = self.create_publisher(Twist, '/motor/main', 5)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.on_receive_scan, 1)
        self.sub_mode = self.create_subscription(String, '/mode', self.on_set_mode, 5)
        self.cli_motor_pos = self.create_client(MotorPos, '/motor/pos')
        ''' timer ''' 
        self.loop_timer = self.create_timer(0.1, self.loop_timer_callback)
        ''' parameter '''
        ''' variable '''
        self.park_dir = ""
        self.yolo_filter = []
        self.last_yolo_result_idx = -1
        self.current_mode = 'boot'
        self.end_cfg = {}
        self.start_time = time.time()
        try:
            file = open('mode_config.json', 'r')
            self.cfg_json = json.load(file)
            self.get_logger().info(f"config:\n{self.cfg_json}")
        except Exception as e:
            self.get_logger().error(f"open mode_config.json failed: {e}")
        self.get_logger().info("init end")
        self.change_mode("init")
        # self.print_menu()
        # self.motor_pos(0.1, 0)
        # self.motor_pos(-0.1, 0)


    def loop_timer_callback(self):
        ## init variables
        new_result = False
        yolo_filted = ""
        ''' try receive yolo result '''
        yolo_result = self.rds.get('yolo_detect')
        yolo_result_idx = self.rds.get('yolo_detect_idx')
        ## check redis result not None
        if yolo_result is not None and yolo_result_idx is not None:
            ## push to filter when is new result
            if yolo_result_idx != self.last_yolo_result_idx:
                yolo_result = json.loads(yolo_result)
                yolo_result = yolo_result['detect']
                for result in yolo_result:
                    prob = result['prob']
                    c_idx = result['c_idx']
                    center_x = result['center_x']
                    center_y = result['center_y']
                    box_w = result['box_w']
                    box_h = result['box_h']
                    ## first filter
                    if prob > 0.4 and box_w * box_h > 5000:
                        self.yolo_filter.append(c_idx)
            ## end when no new result
            else:
                return
        ''' filter '''
        filter_set = set(self.yolo_filter)
        for c_idx in filter_set:
            ## yolo filter class count enougth
            if self.yolo_filter.count(c_idx) > 3:
                self.get_logger().info(f"confirm sign: {class_name[c_idx]}")
                yolo_filted = class_name[c_idx]
                ## clear filter
                self.yolo_filter = []
                break
        ''' end check '''
        for end_cond in self.end_cfg["cond"]:
            ## end condition is sign
            if end_cond[0] == "sign":
                ## change to next mode when match condition target sign
                if yolo_filted == end_cond[1]:
                    self.change_mode(end_cond[2])
            ## end condition is time
            elif end_cond[0] == "time":
                ## change to next mode when reach end time
                if time.time() - self.start_time >= end_cond[1]:
                    self.change_mode(end_cond[2])


    def change_mode(self, mode: str):
        ## exit when mode is same
        if self.current_mode == mode:
            return
        ''' mode change '''
        self.get_logger().info(f"from mode: {self.current_mode} to {mode}")
        ## update mode, clear variable
        self.current_mode = mode
        config = self.cfg_json[mode + '_cfg']
        self.end_cfg = {}
        ## load start config
        start_cfg = config["start"]
        ## start wait
        time.sleep(start_cfg["wait"])
        ## setup trace
        msg = String()
        msg.data = json.dumps(config["trace"])
        self.pub_trace_cfg.publish(msg)
        ## setup avoidance
        msg = String()
        msg.data = json.dumps(config["avoidance"])
        self.pub_avoidance_cfg.publish(msg)
        ## update start time
        self.start_time = time.time()
        ## load end config
        self.end_cfg = config["end"]


    def on_receive_scan(self, msg: LaserScan):
        '''
        handle in parking mode
        '''
        FRONT_ANGLE = 0
        LOCK_DIST = 0.5 # meters
        LOCK_FOV = m.radians(120) / 2 # write fov in ()
        SAMPLES_THRES = 10
        ## parking mode and dir not set
        if self.current_mode == "parking" and self.park_dir == "":
            ## send default vel
            vel = Twist()
            vel.linear.x = 0.05
            self.pub_motor.publish(vel)
            ## init variable
            match_points_count = 0
            match_points_avg_angle = 0
            ## detect match in scan
            for idx in range(len(msg.ranges)):
                ## angle using -pi to +pi
                point_angle = msg.angle_increment * idx
                if point_angle > m.pi:
                    point_angle = -2 * m.pi + point_angle
                ## dist
                point_distance = msg.ranges[idx]
                ## match condition
                if point_distance < LOCK_DIST \
                        and point_angle <= LOCK_FOV \
                        and point_angle >= -LOCK_FOV:
                    match_points_count += 1
                    match_points_avg_angle += point_angle
            ## change mode
            if match_points_count > SAMPLES_THRES:
                if match_points_avg_angle > 0:
                    self.park_dir = "R"
                else:
                    self.park_dir = "L"
                self.get_logger().info(f"park to: {self.park_dir}")
        ## parking mode and dir set
        elif self.current_mode == "parking" and self.park_dir != "":
            self.set_motor_pos(0.5, 0)
            turn = 90
            if self.park_dir == "L":
                turn = -90
            self.set_motor_pos(0, m.radians(turn))
            self.set_motor_pos(-0.2, 0)
            self.set_motor_pos(0.2, 0)
            self.set_motor_pos(0, m.radians(turn))
            self.set_motor_pos(0.5, 0)
            print("end")
            pass


    def set_motor_pos(self, linear: float, angular: float):
        req = MotorPos.Request()
        req.linear = float(linear)
        req.angular = float(angular)
        future = self.cli_motor_pos.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    # def on_set_mode(self, msg: String):
    #     mode_num = msg.data
    #     if mode_num == '0':
    #         self.change_mode('init')
    #     elif mode_num == '1':
    #         self.change_mode('to_fork')
    #     elif mode_num == '2':
    #         self.change_mode('fork_left')
    #     elif mode_num == '3':
    #         self.change_mode('fork_right')
    #     elif mode_num == '4':
    #         self.change_mode('to_avoidance')
    #     elif mode_num == '5':
    #         self.change_mode('avoidance')
    #     elif mode_num == '6':
    #         self.change_mode('to_parking')
    #     elif mode_num == '7':
    #         self.change_mode('approach_parking')


    # def print_menu(self):
    #     print(f"current mode: {self.current_mode}")
    #     print(f"0. init")
    #     print(f"1. to_fork")
    #     print(f"2. fork_left")
    #     print(f"3. fork_right")
    #     print(f"4. to_avoidance")
    #     print(f"5. avoidance")
    #     print(f"6. to_parking")
    #     print(f"7. approach_parking")



def main(args=None):
    rclpy.init(args=args)
    node = ModeSelectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()