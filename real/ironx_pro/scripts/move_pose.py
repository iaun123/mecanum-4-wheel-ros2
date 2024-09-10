#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
import math
from euler_quaternion import euler_from_quaternion

max_linearx = 0.8
max_lineary = 0.8
max_angz = 0.8

def normalize_angle(angle): #angle in degrees
    return (angle + 360) % 360

class RotateRobot(Node):
    def __init__(self):
        super().__init__("move_pose")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(0.5, self.control_loop)
        self.yaw = 0.0 
        self.up_x = 0.0
        self.up_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.target_yaw = None

    def pose_callback(self, data):
        position = data.pose.pose.position
        self.up_x = position.x
        self.up_y = position.y

    def imu_callback(self, data):
        quaternion = data.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

    def control_loop(self):
        self.goal_x = float(sys.argv[1])
        self.goal_y = float(sys.argv[2])
        self.target_yaw = float(sys.argv[3])  # Convert goal yaw to radians
        self.target_yaw = math.radians(normalize_angle(self.target_yaw))
        
        dis_x = self.goal_x - self.up_x
        dis_y = self.goal_y - self.up_y
        distance = math.sqrt(dis_x**2 + dis_y**2)
        angular_error = self.target_yaw - self.yaw
        
        cmd_vel_msg = Twist()
        if distance > 0.005:
            cmd_vel_msg.linear.x = min(max_linearx, max(-max_linearx, 3.0 * dis_x))
            cmd_vel_msg.linear.y = min(max_lineary, max(-max_lineary, 3.0 * dis_y))
            self.get_logger().info("Moving")

        elif abs(angular_error) > 0.0088:  #0.5 deg 
            cmd_vel_msg.angular.z = min(max_angz, max(-max_angz, 3.0 * angular_error))
            self.get_logger().info("Rotating")

        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info("Reached goal")
            quit()

        self.cmd_vel_pub.publish(cmd_vel_msg)
        # print("tyaw = %.3f rad, yaw = %.3f rad" %(math.degrees(self.target_yaw), math.degrees(self.yaw)))
        print("dis = %.3f m, ang = %.3f deg" %(distance, math.degrees(angular_error)))
        print("x = %.3f m/s, y = %.3f m/s, z = %.3f rad/s\n" %(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z))

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