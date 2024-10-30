import os
import time
from dynamixel_sdk import *
import rclpy
from rclpy.node import Node


class UsbDevice:
    def __init__(self):
        pass

    def usb_init(self, usb='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0):
        self.usb = usb
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.portHandler = PortHandler(self.usb)
        self.packetHandler = PacketHandler(self.protocol_version)
        self.set_baudrate(baudrate)
        # Try Open port
        self.open()
        self.close()
        

    def set_baudrate(self, baudrate):
        # Set port baudrate
        if self.portHandler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate {baudrate}")
        else:
            print("Failed to change the baudrate {baudrate}")

    def open(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

    def close(self):
        self.portHandler.closePort()
    
    def ping(self, id):
        model_number, comm_result, error = self.packetHandler.ping(self.portHandler, id)
        if comm_result != COMM_SUCCESS:
            # print("%03d %s" % (id, self.packetHandler.getTxRxResult(comm_result)))
            pass
        elif error != 0:
            # print("%03d %s" % (id, self.packetHandler.getRxPacketError(error)))
            pass
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, model_number))


class FindMotor(Node):
    def __init__(self):
        super().__init__('find_motor')
        self.baudrate_list = [9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000]
        self.usb = '/dev/ttyUSB0'
        self.motor = UsbDevice()
        self.motor.usb_init(usb = self.usb)
        self.declare_parameter('act', [0])
        act = self.get_parameter('act').get_parameter_value().integer_array_value
        print(act)
        if len(act) == 4:
            # act_list = eval(act) # src_id, src_baudrate, dst_id, dst_baudrate
            self.change(act[0], act[1], act[2], act[3])
        else:
            # Find Motor
            for baud in self.baudrate_list:
                print(f"try baudrate: {baud}")
                self.motor.set_baudrate(baud)
                self.ping_scan()


    def change(self, src_id, src_baudrate, dst_id, dst_baudrate):
        dst_baudrate_idx = self.baudrate_list.index(dst_baudrate)
        print(f"try change from id: {src_id} baudrate: {src_baudrate}" \
            f" to id: {dst_id} baudrate: {dst_baudrate} {dst_baudrate_idx}")
        # in src baudrate + src id
        self.motor.set_baudrate(src_baudrate)
        self.motor.open()
        self.motor.ping(src_id)
        self.motor.packetHandler.write1ByteTxRx(self.motor.portHandler, src_id, 8, dst_baudrate_idx)
        self.motor.close()
        # in dst baudrate + src_id
        self.motor.set_baudrate(dst_baudrate)
        self.motor.open()
        self.motor.packetHandler.write1ByteTxRx(self.motor.portHandler, src_id, 7, dst_id)
        self.motor.ping(dst_id)
        self.motor.close()
        

    def ping_scan(self):
        self.motor.open()
        for id in range(0, 252 + 1):
            self.motor.ping(id)
        self.motor.close()


def main(args=None):
    rclpy.init(args=args)
    motor_node = FindMotor()
    # rclpy.spin(motor_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

