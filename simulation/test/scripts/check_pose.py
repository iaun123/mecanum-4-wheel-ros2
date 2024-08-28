#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from euler_quaternion import euler_from_quaternion

class PoseNode(Node):
    def __init__(self):
        super().__init__('position')
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Initialize position and yaw
        self.up_x = None
        self.up_y = None
        self.yaw = None

    def odom_callback(self, data):
        position = data.pose.pose.position
        self.up_x = position.x
        self.up_y = position.y

    def imu_callback(self, imu):
        quaternion = imu.orientation
        roll, pitch, self.yaw = euler_from_quaternion(quaternion)

    def log_pose(self):
        if self.up_x is not None and self.up_y is not None and self.yaw is not None:
            yaw_degrees = math.degrees(self.yaw)
            yaw_degrees= (yaw_degrees + 360) % 360
            print(f"x = {self.up_x:.3f} m, y = {self.up_y:.3f} m, yaw = {yaw_degrees:.3f} deg\n")
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if node.log_pose():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
