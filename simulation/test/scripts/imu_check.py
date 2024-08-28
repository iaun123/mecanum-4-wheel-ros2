#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from euler_quaternion import euler_from_quaternion
import math

class IMUPrinterNode(Node):
    def __init__(self):
        super().__init__('imu_printer')
        self.subscription = self.create_subscription(Imu,'imu',self.imu_callback,10 )
        
    def imu_callback(self, msg: Imu):
        quaternion = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = math.degrees(yaw)
        print("yaw =  %.3f deg" % (yaw))
        

def main(args=None):
    rclpy.init(args=args)
    imu_printer = IMUPrinterNode()
    rclpy.spin(imu_printer)
    imu_printer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
