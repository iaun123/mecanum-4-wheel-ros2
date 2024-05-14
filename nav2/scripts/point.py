#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import math
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    point = input("Enter a point : ")
    
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    if point == 'a' :
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = -3.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.1

        navigator.goToPose(goal_pose)
        i = 0

        while not navigator.isNavComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                navigator.cancelNav()

        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        exit(0)
    
    if point == 'b' :
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 3.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.1
        
        navigator.goToPose(goal_pose)
        i = 0

        while not navigator.isNavComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                navigator.cancelNav()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        exit(0)

    else:
        print('no point')
        exit(0)


if __name__ == '__main__':
    main()