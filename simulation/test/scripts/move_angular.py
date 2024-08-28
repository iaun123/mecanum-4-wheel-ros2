#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import sys
import math
from euler_quaternion import euler_from_quaternion

class RotateRobot(Node):
    def __init__(self):
        super().__init__("move_angular")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.rotate_to_yaw)
        self.yaw = 0.0  # Initialize yaw
        self.target_yaw = None

    def imu_callback(self, data):
        quaternion = data.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        # self.get_logger().info(f"IMU Callback: roll = {self.roll}, pitch = {self.pitch}, yaw = {self.yaw}")

    def rotate_to_yaw(self):
        if self.target_yaw is None:
            self.target_yaw = float(sys.argv[1])
            self.target_yaw = math.radians(self.target_yaw)  # Convert goal yaw to radians

        angular_error = self.target_yaw - self.yaw

        # Normalize the angular error to be within the range -pi to pi
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        ang_deg = math.degrees(angular_error)  # For logging
        print('')
        print("yaw = %.3f rad, Target = %.3f rad, error = %.3f rad" % (self.yaw, self.target_yaw, angular_error))

        cmd_vel_msg = Twist()
        
        if abs(angular_error) > 0.05:  # rad
            cmd_vel_msg.angular.z = 10.0 * angular_error  # Scaled angular velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.get_logger().info("Rotating")
        else:
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.get_logger().info("Goal yaw reached")
            quit()

        print("cmd = %.3f rad/s" % (cmd_vel_msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
