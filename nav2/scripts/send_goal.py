#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import math
from robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  yaw = yaw * math.pi/ 180
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def main():
    rclpy.init()

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_z = 0.0
    goal_roll = 0.0
    goal_pitch = 0.0
    goal_yaw = float(sys.argv[3]) # deg
    x,y,z,w = get_quaternion_from_euler(goal_roll,goal_pitch,goal_yaw)
    # print(f"X = {goal_x} Y = {goal_y} Z = {goal_z} x = {x} y = {y} z = {z} w = {w}")

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Set the robot's goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.position.z = goal_z
    goal_pose.pose.orientation.x = x
    goal_pose.pose.orientation.y = y
    goal_pose.pose.orientation.z = z
    goal_pose.pose.orientation.w = w

    # Go to the goal pose
    navigator.goToPose(goal_pose)
    i = 0

    # Keep doing stuff as long as the robot is moving towards the goal
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

if __name__ == '__main__':
    main()