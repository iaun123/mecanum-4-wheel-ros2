#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
import math
import csv
from euler_quaternion import euler_from_quaternion

max_linearx = 0.8
max_lineary = 0.8
max_angz = 0.8

def normalize_angle(angle):  # angle in degrees
    return (angle + 360) % 360

class RotateRobot(Node):
    def __init__(self):
        super().__init__("move_pose")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(0.5, self.control_loop)
        self.yaw = 0.0
        self.up_x = 0.0
        self.up_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.target_yaw = None

        # Open CSV file in append mode and set up CSV writer
        # self.csv_file = open('pose_log.csv', 'a', newline='')
        # self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header only if the file is empty
        # if self.csv_file.tell() == 0:
        #     self.csv_writer.writerow(['x (m)', 'y (m)', 'yaw (deg)', 'distance (m)', 'angular_error (deg)'])

    def pose_callback(self, data):
        position = data.pose.pose.position
        self.up_x = position.x
        self.up_y = position.y

    def imu_callback(self, data):
        quaternion = data.orientation
        _, _, self.yaw = euler_from_quaternion(quaternion)

    def control_loop(self):
        if self.goal_x is None or self.goal_y is None or self.target_yaw is None:
            # Initialize goal parameters from sys.argv
            self.goal_x = float(sys.argv[1])
            self.goal_y = float(sys.argv[2])
            self.target_yaw = float(sys.argv[3])
            self.target_yaw = math.radians(normalize_angle(self.target_yaw))
        
        dis_x = self.goal_x - self.up_x
        dis_y = self.goal_y - self.up_y
        distance = math.sqrt(dis_x**2 + dis_y**2)
        angular_error = self.target_yaw - self.yaw
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        
        cmd_vel_msg = Twist()
        if distance > 0.005:
            cmd_vel_msg.linear.x = min(max_linearx, max(-max_linearx, 3.0 * dis_x))
            cmd_vel_msg.linear.y = min(max_lineary, max(-max_lineary, 3.0 * dis_y))
            self.get_logger().info("Moving")

        elif abs(angular_error) > 0.05 :  # 0.5 degrees 0.0088
            cmd_vel_msg.angular.z = min(max_angz, max(-max_angz, 3.0 * angular_error))
            self.get_logger().info("Rotating")

        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info("Reached goal")
            print(f"x = {self.up_x:.3f} m, y = {self.up_y:.3f} m, yaw = {self.yaw:.3f}")
            exit(0)

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.log_pose(distance, angular_error)
        
    def log_pose(self, distance, angular_error):
        # Get current timestamp
        # timestamp = self.get_clock().now().to_msg()
        # Convert yaw from radians to degrees
        yaw_degrees = math.degrees(self.yaw)
        yaw_degrees = (yaw_degrees + 360) % 360

        # Write the data to CSV file
        # self.csv_writer.writerow([f"{self.up_x:.3f}",f"{self.up_y:.3f}",f"{yaw_degrees:.3f}",])

    # def stop_and_exit(self):
    #     # Stop robot and exit
    #     stop_msg = Twist()
    #     self.cmd_vel_pub.publish(stop_msg)
    #     self.destroy_node()
    #     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit(0)

if __name__ == '__main__':
    main()
