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
        self.subscription = self.create_subscription(
            Path,'/plan',self.global_path_callback,10)

        # Waypoints setup
        self.goal_poses = [self.create_goal_pose(waypoint) for waypoint in waypoints]
        self.current_index = 0
        self.path_saved = False
        self.navigator.goToPose(self.goal_poses[self.current_index])
        self.create_timer(1, self.check_feedback)

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

    def global_path_callback(self, path_msg):
        if not self.path_saved and self.current_index < len(self.goal_poses):
            path_data = []
            for pose_stamped in path_msg.poses:
                position = pose_stamped.pose.position
                orientation = pose_stamped.pose.orientation
                roll, pitch, yaw = euler_from_quaternion(orientation)
                yaw_degrees = math.degrees(yaw)
                yaw_degrees = (yaw_degrees + 360) % 360  # Normalize to [0, 360) degrees

                # Store the global path waypoints in a list
                path_data.append([f"{position.x:.3f}", f"{position.y:.3f}", f"{yaw_degrees:.3f}"])

            # Save the path data to a CSV file
            filename = f'global_path_{self.current_index+1}.csv'
            with open(filename, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(['x', 'y', 'yaw'])  # Header
                csvwriter.writerows(path_data)

            self.get_logger().info(f'Global path saved to {filename}')
            self.path_saved = True  # Mark the path as saved

    def check_feedback(self):
        feedback = self.navigator.getFeedback()
        if feedback:
            current_pose = feedback.current_pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            roll, pitch, current_yaw = euler_from_quaternion(current_pose.orientation)
            current_yaw = math.degrees(current_yaw)
            current_yaw = (current_yaw + 360) % 360

            # Log the current pose to the CSV file
            point_name = ['WP1', 'WP2', 'WP3'][self.current_index]

        if self.navigator.isNavComplete():
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                self.get_logger().info(f'Successfully reached {point_name} : x = {current_x:.3f}, y = {current_y:.3f}, yaw = {current_yaw:.3f}\n')
                self.current_index += 1
                if self.current_index < len(self.goal_poses):
                    self.path_saved = False  # Reset path saved flag for the next waypoint
                    self.navigator.goToPose(self.goal_poses[self.current_index])
                else:
                    self.get_logger().info('All goals succeeded!')
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
