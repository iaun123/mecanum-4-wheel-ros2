#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
import math
import csv
from euler_quaternion import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path

class Nav2move(Node):
    def __init__(self, waypoints):
        super().__init__("nav2_waypoints")
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Waypoints setup
        self.goal_poses = [self.create_goal_pose(waypoint) for waypoint in waypoints]
        self.current_index = 0
        self.navigator.goToPose(self.goal_poses[self.current_index])
        self.create_timer(0.8, self.check_feedback)

        # Initialize pose variables
        self.up_x = 0.0
        self.up_y = 0.0
        self.yaw = 0.0

        # Open CSV file in write mode and set up CSV writer
        self.csv_file = open('pose_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header
        self.csv_writer.writerow(['Waypoint', 'x (m)', 'y (m)', 'yaw (deg)'])

    def create_goal_pose(self, point):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        points = {
            'a': (4.0, 0.0, 0.0),
            'b': (-1.0, 3.0, 45.0),
            'c': (0.0, -4.0, 315.0),
        }

        if point in points:
            x, y, yaw = points[point]
            yaw = math.radians(yaw)
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
        else:
            self.get_logger().error(f'Invalid point: {point}')
            rclpy.shutdown()
            exit(0)
        return goal_pose

    def check_feedback(self):
        feedback = self.navigator.getFeedback()
        if feedback:
            current_pose = feedback.current_pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            roll, pitch, current_yaw = euler_from_quaternion(current_pose.orientation)
            current_yaw = math.degrees(current_yaw)
            current_yaw = (current_yaw + 360) % 360

            point_name = ['WP1', 'WP2', 'WP3'][self.current_index]
            self.csv_writer.writerow([point_name, f"{current_x:.3f}", f"{current_y:.3f}", f"{current_yaw:.3f}"])

        if self.navigator.isNavComplete():
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print(f'Successfully reached {point_name} : x = {current_x:.3f}, y = {current_y:.3f}, yaw = {current_yaw:.3f}\n')
                self.current_index += 1
                if self.current_index < len(self.goal_poses):
                    self.navigator.goToPose(self.goal_poses[self.current_index])
                else:
                    self.get_logger().info('All goals succeeded!')
                    self.csv_file.close()  # Close the CSV file when done
                    exit(0)
            elif result == NavigationResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
                self.csv_file.close()
                exit(0)
            elif result == NavigationResult.FAILED:
                self.get_logger().info('Goal failed!')
                self.csv_file.close()
                exit(0)
            else:
                self.get_logger().info('Goal has an invalid return status!')
                self.csv_file.close()
                exit(0)

def main():
    rclpy.init()
    input_str = input("Enter 3 points a-c ('a b c'): ")
    waypoints = input_str.split()
    if len(waypoints) != 3:
        print('You must enter exactly 3 points.')
        rclpy.shutdown()
        exit(0)
    
    node = Nav2move(waypoints)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
