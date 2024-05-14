#!/usr/bin/env python3
import rclpy
import sys
import math

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
    goal_roll = 0.0
    goal_pitch = 0.0
    goal_yaw = float(sys.argv[1]) # deg
    x,y,z,w = get_quaternion_from_euler(goal_roll,goal_pitch,goal_yaw)
    print(f"x = {x} y = {y} z = {z} w = {w}")

if __name__ == '__main__':
    main()