#! /usr/bin/env python3

import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
import rclpy

from robot_navigator import BasicNavigator, NavigationResult

'''
Basic navigation demo to go to poses.
''' 
def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.w = 0.1
    # initial_pose.pose.orientation.z = 0.0

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set our demo's goal poses to follow
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 0.1
    goal_pose1.pose.orientation.z = 0.0
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.0
    goal_pose2.pose.position.y = -1.0
    goal_pose2.pose.orientation.w = 0.1
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -2.0
    goal_pose3.pose.position.y = 0.0
    goal_pose3.pose.orientation.w = 0.1
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

    nav_start = navigator.get_clock().now()

    navigator.followWaypoints(goal_poses)
    #navigator.goToPose(goal_poses[0])
    
    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

    
    # Do something depending on the return code
    result = navigator.getResult()
    print("navigator.getResult(): ", navigator.getResult())
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
