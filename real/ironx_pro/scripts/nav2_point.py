#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
import math
from euler_quaternion import quaternion_from_euler
import sys
from rclpy.duration import Duration

class Nav2move(Node):
    def __init__(self, goal_x, goal_y, goal_yaw):
        super().__init__("nav2_point")
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
        if self.navigator.isNavComplete():
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
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
           
        # feedback = self.navigator.getFeedback()
        # if feedback:
        #     print(f'Distance: {feedback.distance_remaining:.2f} m')
        #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
        #         self.navigator.cancelNav()

def main():
    rclpy.init()
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_yaw = float(sys.argv[3])  # deg
    goal_yaw = math.radians(goal_yaw)  # Convert goal yaw to radians

    node = Nav2move(goal_x, goal_y, goal_yaw)
    rclpy.spin(node)
    exit(0)

if __name__ == '__main__':
    main()
