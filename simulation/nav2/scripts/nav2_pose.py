#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import csv
from euler_quaternion import quaternion_from_euler, euler_from_quaternion

class Nav2move(Node):
    def __init__(self, goal_x, goal_y, goal_yaw):
        super().__init__("nav2_pose")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.target_yaw = goal_yaw
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.set_goal_pose()

        # Open CSV files in append mode and set up CSV writers
        # self.csv_file = open('a5_pose.csv', 'a', newline='')
        # self.csv_writer = csv.writer(self.csv_file)
        
        # self.csv_writer.writerow(['current_x (m)', 'current_y (m)', 'current_yaw (deg)'])

        self.timer = self.create_timer(1.0, self.check_feedback) 
        self.start_time = self.navigator.get_clock().now()

    def set_goal_pose(self):
        q = quaternion_from_euler(0.0, 0.0, self.target_yaw)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = self.goal_x
        self.goal_pose.pose.position.y = self.goal_y
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = q[0]
        self.goal_pose.pose.orientation.y = q[1]
        self.goal_pose.pose.orientation.z = q[2]
        self.goal_pose.pose.orientation.w = q[3]
        self.navigator.goToPose(self.goal_pose)

    def check_feedback(self):
        feedback = self.navigator.getFeedback()
        if feedback:
            current_pose = feedback.current_pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            roll, pitch, current_yaw = euler_from_quaternion(current_pose.orientation)
            current_yaw = math.degrees(current_yaw)
            current_yaw = (current_yaw + 360) % 360

        if self.navigator.isNavComplete():
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                if feedback:
                    self.log_pose(feedback.current_pose.pose)
                    print(f'Successfully reached: x = {current_x:.3f}, y = {current_y:.3f}, yaw = {current_yaw:.3f}\n')
                exit(0)
            elif result == NavigationResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
                exit(0)
            elif result == NavigationResult.FAILED:
                self.get_logger().info('Goal failed!')
                exit(0)
            else:
                self.get_logger().info('Goal has an invalid return status!')
                exit(0)
        elif feedback:
            self.log_pose(feedback.current_pose.pose)

    def log_pose(self, pose):
        current_x = pose.position.x
        current_y = pose.position.y
        roll, pitch, current_yaw = euler_from_quaternion(pose.orientation)
        current_yaw_degrees = math.degrees(current_yaw)
        current_yaw_degrees = (current_yaw_degrees + 360) % 360

        # Write the current pose to CSV file with three decimal places
        # self.csv_writer.writerow([f"{current_x:.3f}", f"{current_y:.3f}", f"{current_yaw_degrees:.3f}"])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    point = input("Enter a point (a-c): ")
    points = {
        'a': (4.0, 0.0, 0.0),
        'b': (-1.0, 3.0, 45.0),
        'c': (0.0, -4.0, 315.0),
    }

    if point not in points:
        print('Invalid point')
        rclpy.shutdown()
        exit(0)

    goal_x, goal_y, goal_yaw = points[point]
    goal_yaw = (goal_yaw + 360) % 360
    goal_yaw = math.radians(goal_yaw)  

    node = Nav2move(goal_x, goal_y, goal_yaw)
    rclpy.spin(node)
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
