#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math
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
        
        self.timer = self.create_timer(0.5, self.check_feedback)  # Timer to check feedback periodically
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
        if self.navigator.isNavComplete():
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                # self.get_logger().info('Goal succeeded!')
                
                if feedback:
                    current_pose = feedback.current_pose.pose
                    current_x = current_pose.position.x
                    current_y = current_pose.position.y
                    roll, pitch, current_yaw = euler_from_quaternion(current_pose.orientation)

                    # Convert yaw from radians to degrees
                    current_yaw_degrees = math.degrees(current_yaw)
                    current_yaw_degrees = (current_yaw_degrees + 360) % 360

                    # Print the current pose
                    print(f'Successfully reached: x = {current_x:.3f}, y = {current_y:.3f}, yaw = {current_yaw_degrees:.3f} deg')
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
    point = input("Enter a point (a-c): ")
    points = {
        'a': (-1.45, 0.6, 0.0),
        'b': (-1.2, 1.55, 0.0),
        'c': (-0.6, 3.0, 0.0),
}

    if point not in points:
        print('Invalid point')
        rclpy.shutdown()
        exit(0)

    goal_x, goal_y, goal_yaw = points[point]
    goal_yaw = (goal_yaw + 360) % 360
    goal_yaw = math.radians(goal_yaw)  # Convert goal yaw to radians

    node = Nav2move(goal_x, goal_y, goal_yaw)
    rclpy.spin(node)
    rclpy.shutdown()  # Ensure ROS is properly shut down after node execution

if __name__ == '__main__':
    main()
