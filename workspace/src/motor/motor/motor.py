import os
import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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

    def set_baudrate(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
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

    def ping(self):
        m1_model_number, m1_comm_result, m1_error = self.packetHandler.ping(self.portHandler, self.m1_id)
        if m1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m1_comm_result))
        elif m1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m1_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m1_id, m1_model_number))

        m2_model_number, m2_comm_result, m2_error = self.packetHandler.ping(self.portHandler, self.m2_id)
        if m2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m2_comm_result))
        elif m2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m2_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m2_id, m2_model_number))

    def set_speed(self, m1_speed, m2_speed):
        # Motor 1
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m1_id, 104, int(m1_speed))
        # Motor 2
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m2_id, 104, int(m2_speed))



class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            Twist,
            '/motor',
            self.set_target_callback,
            10)

    def set_target_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imshow("receive", img)
        L_min, debug_img = self.get_trace_value(img)
        self.get_logger().info(f"L: {L_min}")
        cv2.imshow("traceline L", debug_img)
        cv2.waitKey(1)



def init_motor() -> MOTOR_2_WHEEL_MODE:
    motor = MOTOR_2_WHEEL_MODE()
    #motor.usb_initialization(usb='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TFQFM-if00-port0', baudrate=1000000, protocol_version=2.0)
    motor.usb_initialization(usb='/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00')
    motor.motor_initialization(m1_id=1, m2_id=2)
    motor.ping()
    motor.setSpeed(0, 0)
    return motor



if __name__ == '__main__':
    motor = MOTOR_2_WHEEL_MODE()
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


