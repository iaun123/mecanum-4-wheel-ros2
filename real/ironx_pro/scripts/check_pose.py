#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
from euler_quaternion import euler_from_quaternion

class PoseNode(Node):
    def __init__(self):
        super().__init__('position')
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.timer = self.create_timer(0.5, self.log_pose)

    def odom_callback(self, data):
        position = data.pose.pose.position
        self.up_x = position.x
        self.up_y = position.y
        
    def imu_callback(self, imu):
        quaternion = imu.orientation
        roll, pitch, self.yaw = euler_from_quaternion(quaternion)

    def log_pose(self):
        yaw_degrees = math.degrees(self.yaw)
        print(f"x = {self.up_x:.3f} m, y = {self.up_y:.3f} m, yaw = {yaw_degrees:.3f} deg\n")

def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
