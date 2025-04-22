import os
import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Int8
from motor_if.srv import MotorPos
import math as m
import numpy as np
from nav_msgs.msg import Odometry
import tf2_ros 
from tf_transformations import quaternion_from_euler

motor_obj = None 


class UsbDevice:
    def __init__(self):
        pass

    def usb_init(self, usb='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0):
        self.usb = usb
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.portHandler = PortHandler(self.usb)
        self.packetHandler = PacketHandler(self.protocol_version)
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()
        # Set baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def close(self):
        self.portHandler.closePort()


class Motor2Wheel(UsbDevice):
    def __init__(self):
        pass

    def motor_init(self, m1_id=1, m2_id=2):
        # Save ID
        self.m1_id = m1_id
        self.m2_id = m2_id
        # Set Position Limit
        # DXL_MINIMUM_POSITION_VALUE  = 0   # Refer to the Minimum Position Limit of product eManual
        # DXL_MAXIMUM_POSITION_VALUE  = 4095    # Refer to the Maximum Position Limit of product eManual
        # dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
        # Motor Init
        # m1_comm_result, m1_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m1_id, 48, dxl_goal_position[0])
        # m2_comm_result, m2_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m2_id, 48, dxl_goal_position[0])
        # Set Velocity mode
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 11, 1)
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 11, 1)
        # Torque Enable
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 64, 1)
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 64, 1)

    def has_error(self, comm_result, error) -> bool:
        '''
            return: false when success
        '''
        if comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(comm_result))
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
        else:
            return False
    
    def ping(self):
        m1_model_number, comm_result, error = self.packetHandler.ping(self.portHandler, self.m1_id)
        if(not self.has_error(comm_result, error)):
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m1_id, m1_model_number))
        m2_model_number, comm_result, error = self.packetHandler.ping(self.portHandler, self.m2_id)
        if(not self.has_error(comm_result, error)):
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m2_id, m2_model_number))

    def set_speed(self, m1_speed, m2_speed):
        '''
            speed: rpm
        '''
        ## factor 0.229 from XM430-W350 datasheet
        # Motor 1
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m1_id, 104, int(m1_speed / 0.229))
        # Motor 2
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m2_id, 104, int(m2_speed / 0.229))
    
    def get_speed(self) -> list[float]:
        '''
            speed: rpm \n
            return: list[m1, m2]
        '''
        ret_vel = [0.0] * 2
        m1_vel_raw, comm_result, error = self.packetHandler.read4ByteTxRx(self.portHandler, self.m1_id, 112)
        if(not self.has_error(comm_result, error)):
            ret_vel[0] = m1_vel_raw * 0.229
        m2_vel_raw, comm_result, error = self.packetHandler.read4ByteTxRx(self.portHandler, self.m2_id, 112)
        if(not self.has_error(comm_result, error)):
            ret_vel[1] = m2_vel_raw * 0.229
        return ret_vel



class MotorNode(Node):
    def __init__(self):
        global motor_obj
        super().__init__('motor_node')
        self.sub_motor_main = self.create_subscription(
            Twist,
            '/motor/main',
            self.receive_motor_main_callback,
            1)
        self.sub_motor_aux = self.create_subscription(
            Twist,
            '/motor/aux',
            self.receive_motor_aux_callback,
            1)
        self.srv_motor_pos = self.create_service(
            MotorPos, 
            '/motor/pos', 
            self.motor_pos_callback)
        self.pub_odom = self.create_publisher(
            Odometry,
            '/odom',
            1
        )
        ''' Car Struct '''
        self.wheel_radius = 0.0325 # meters
        self.wheel_dist = 0.162 # meters
        self.wheel_perimeter = 2 * m.pi * self.wheel_radius # meters
        ''' Variable '''
        self.main_linear = 0
        self.main_angular = 0
        self.aux_angular = 0
        ''' Motor '''
        self.motor = Motor2Wheel() # left: m1, right: m2
        motor_obj = self.motor
        self.motor.usb_init(usb='/dev/ttyUSB0')
        self.motor.motor_init(m1_id=1, m2_id=2)
        self.motor.ping()
        self.motor.set_speed(0, 0)
        time.sleep(3)
        self.get_logger().info("init finish")
        ''' Odom '''
        self.odom_last_time = 0
        self.last_vel = np.array([0, 0], dtype=np.float32) # L, R
        self.odom_xy = np.array([0, 0], dtype=np.float32) # x, y
        self.odom_theta = 0
        self.odom_timer = self.create_timer(0.01, self.odom_timer_callback)
        ''' debug '''


    def receive_motor_main_callback(self, msg: Twist):
        linear = msg.linear.z # m/s
        angular = msg.angular.z # rad/s
        self.get_logger().info(f"twist main: linear-> {linear}, angular-> {angular}")
        self.main_linear = linear
        self.main_angular = angular
        self.set_target()


    def receive_motor_aux_callback(self, msg: Twist):
        angular = msg.angular.z # rad/s
        self.get_logger().info(f"twist aux: angular-> {angular}")
        self.aux_angular = angular
        self.set_target()


    def motor_pos_callback(self, request, response):
        REF_LINEAR = 0.1 # m/s
        REF_ANGULAR = 1 # rad/s
        linear = request.linear # m/s
        angular = request.angular # rad/s
        self.get_logger().info(f"twist pos: linear-> {linear}, angular-> {angular}")
        if linear and angular:
            self.get_logger().warn(f"twist pos input linear and angular at same time")
        self.aux_angular = self.main_angular = self.main_linear = 0
        if angular:
            dt = angular / REF_ANGULAR
            if dt >= 0:
                self.main_angular = REF_ANGULAR
            else:
                self.main_angular = -REF_ANGULAR
            self.set_target()
            time.sleep(abs(dt))
            self.main_angular = 0
            self.set_target()
        elif linear:
            dt = linear / REF_LINEAR
            if dt >= 0:
                self.main_linear = REF_LINEAR
            else:
                self.main_linear = -REF_LINEAR
            self.set_target()
            time.sleep(abs(dt))
            self.main_linear = 0
            self.set_target()
        ## pub status
        response.result = 1
        return response


    def set_target(self):
        linear = self.main_linear # m/s
        angular = self.main_angular + self.aux_angular # rad/s
        self.get_logger().info(f"twist target: linear-> {linear}, angular-> {angular}")
        speed_left = linear - (self.wheel_dist / 2) * angular
        speed_right = linear + (self.wheel_dist / 2) * angular
        output_left = 60 * speed_left / (self.wheel_perimeter)     # mult 60 for rps to rpm
        output_right = 60 * speed_right / (self.wheel_perimeter)   # mult 60 for rps to rpm
        self.get_logger().info(f"output target: left: {output_left}, right: {output_right}")
        self.motor.set_speed(output_left, output_right)
    
    def odom_timer_callback(self):
        ## inital set time
        if self.odom_last_time == 0:
            self.odom_last_time = time.time()
            return
        ## get dt
        dt = time.time() - self.odom_last_time
        self.odom_last_time = time.time()
        ## get linear vel (m/s)
        vel_rpm = self.motor.get_speed()
        vel = np.array(vel_rpm) / 60.0 * self.wheel_perimeter
        avg_vel = (vel + self.last_vel) / 2
        ## get delta distance
        delta_distance_lr = avg_vel * dt
        delta_distance_ref = (delta_distance_lr[0] + delta_distance_lr[1]) * 0.5
        ## odom (seq is matter)
        delta_theta = (delta_distance_ref[1] - delta_distance_ref[1]) / self.wheel_dist
        self.odom_xy[0] +=  delta_distance_ref * m.cos(self.odom_theta + delta_theta / 2)
        self.odom_xy[1] +=  delta_distance_ref * m.sin(self.odom_theta + delta_theta / 2)
        self.odom_theta += delta_theta
        ## update last
        self.last_vel = vel
        ## publish
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now()
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.odom_xy[0])
        msg.pose.pose.position.y = float(self.odom_xy[1])
        msg.pose.pose.position.z = 0.0
        q_raw = quaternion_from_euler(0, 0, self.odom_theta)
        quat = Quaternion()
        quat.w = q_raw[0]
        quat.x = q_raw[1]
        quat.y = q_raw[2]
        quat.z = q_raw[3]
        msg.pose.pose.orientation = quat
        msg.twist.twist.angular.z = delta_theta / self.wheel_dist
        msg.twist.twist.linear.x = delta_distance_ref / self.wheel_dist
        self.pub_odom.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    try:
        rclpy.spin(motor_node)
    except:
        motor_obj.set_speed(0, 0)
    rclpy.shutdown()




if __name__ == '__main__':
    motor = Motor2Wheel()
    motor.usb_initialization(usb='/dev/ttyACM0')
    motor.motor_initialization(m1_id=1, m2_id=2)
    motor.ping()
    motor.setSpeed(0, 0)
    # motor.setSpeed(100, 100)
    # time.sleep(1)
    # motor.setSpeed(-50, -50)
    # time.sleep(2)
    #motor.setSpeed(-100, -100)
    #time.sleep(1)
    #motor.goDist(150, 500)
    #motor.goDist(-50, 50)
    #motor.goRotate(90, 50)
    #motor.goRotate(-90, 50)
    # while 1:
    #     print('go')
    # motor.setSpeed(200, 200)
    # time.sleep(1)
    # print('stop')
    # motor.setSpeed(0, 0)
    # time.sleep(1)


