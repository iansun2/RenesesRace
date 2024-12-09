import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import math as m

class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.get_logger().info("init start")
        ''' config '''
        # angle of head(front)
        self.head_position = m.radians(0)
        # avoidance fov (half)
        self.avoidance_fov = m.radians(180) / 2 # write fov in ()
        # weight at avoidance_angle edge
        self.avoidance_angle_weight_min = 0.1
        self.avoidance_fov_factor = (1 - self.avoidance_angle_weight_min) / self.avoidance_fov
        # start avoidance distance
        self.avoidance_distance = 0.4
        self.distance_kp = 0.1
        ''' formula
        avoidance_fov_factor = (1 - avoidance_angle_weight_min) / avoidance_fov
        point_weight = 1 - avoidance_fov_factor * (point_angle - head_position)
        weighted_sum = sum( distance_kp * (avoidance_distance - point_distance) * point_weight )
        final_output = weighted_sum
        '''
        ''' pubsub '''
        self.pub_mot = self.create_publisher(Twist, '/motor_sup', 1)
        self.pub_debug = self.create_publisher(LaserScan, '/avoidance_debug', 1)
        # self.sub_ctrl = self.create_subscription(String, '/avoidance_ctrl', self.on_receive_ctrl, 1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_receive_scan, 1)
        ''' end '''
        self.get_logger().info("init finish")


    ''' receive lidar scan callback '''
    def on_receive_scan(self, msg):
        self.get_logger().info("scan receive")
        self.get_logger().info(f"ang_min: {msg.angle_min}")
        self.get_logger().info(f"ang_max: {msg.angle_max}")
        self.get_logger().info(f"ang_inc: {msg.angle_increment}")
        self.get_logger().info(f"point cnt: {len(msg.ranges)}")
        weighted_sum = 0
        weighted_value = []
        for idx in range(len(msg.ranges)):
            # angle
            point_angle = msg.angle_increment * idx
            if point_angle > m.pi:
                point_angle = -2 * m.pi + point_angle
            # dist
            point_distance = msg.ranges[idx]
            weighted_value.append(0.0)
            # filt point
            if point_angle > self.head_position + self.avoidance_fov \
                    or point_angle < self.head_position - self.avoidance_fov \
                    or point_distance > self.avoidance_distance:
                continue
            point_weight = self.avoidance_fov_factor * (point_angle - self.head_position)
            if point_weight >= 0:
                point_weight = 1 - point_weight
            else:
                point_weight = -1 - point_weight
            weighted_value[-1] = self.distance_kp * (self.avoidance_distance - point_distance) * point_weight
            weighted_sum += weighted_value[-1]
            ## for debug
            weighted_value[-1] = abs(weighted_value[-1]) * 30
            # weighted_value[-1] = 1 + point_weight
            # print(f"pw: {point_weight}")
        final_output = weighted_sum
        self.get_logger().info(f"final output: {final_output}")
        # for debug
        msg.ranges = weighted_value
        self.pub_debug.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    avoidance_node = AvoidanceNode()
    rclpy.spin(avoidance_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()