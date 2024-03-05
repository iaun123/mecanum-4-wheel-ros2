#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Goal(Node):
    def __init__(self):
        super().__init__("Go_to_Goal")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, 'odom', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Odometry()
    
    def cal_error(self, goal_x, goal_y, goal_yaw):
        dis_x = goal_x - self.up_x
        dis_y = goal_y - self.up_y
        distance = math.sqrt(math.pow((dis_x), 2) + math.pow((dis_y), 2))
        angle = math.atan2(dis_y, dis_x) 
        ang_yaw = goal_yaw - self.up_yaw

        #max_speed
        # cmd_x = 0.8 #m/s
        # cmd_y = 0.8
        # cmd_z = 0.8 #rad/s

        sp_x = 0.2 #m/s
        sp_y = 0.2
        sp_yaw = 0.2 #rad/s

        return sp_x, sp_y, sp_yaw, distance, ang_yaw

    def pose_callback(self, data):
        self.pose = data
        self.up_x = self.pose.pose.pose.position.x
        self.up_y = self.pose.pose.pose.position.y
        # self.up_yaw = self.pose.pose.pose.position.z
        roll, pitch, self.up_yaw = euler_from_quaternion(self.pose.pose.pose.orientation)
        # print(f"x = {self.up_x} y = {self.up_y} yaw = {self.up_yaw*180/math.pi}")

    def go_to_goal(self):
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_yaw = float(sys.argv[3]) # deg
        goal_yaw = goal_yaw * math.pi / 180

        x, y, yaw, dis, ang = self.cal_error(goal_x, goal_y, goal_yaw)
        # print("cmd_x = %.3f, cmd_y = %.3f, cmd_z = %.3f" % (x, y, yaw))
        print(f"x = {self.up_x} y = {self.up_y} yaw = {self.up_yaw*180/math.pi}")
        print("dis = %.3f  , ang = %.3f" % (dis, ang)) 

        if (self.up_x < goal_x): 
            x = 0.2
        elif (self.up_x > goal_x): 
            x = -0.2

        if (self.up_y < goal_y): 
            y = 0.2
        elif (self.up_y > goal_y): 
            y = -0.2

        if (self.up_yaw < goal_yaw):
            yaw = 0.2
        elif (self.up_yaw > goal_yaw):
            yaw = -0.2

        distance_tolerance = 0.1
        angle_tolerance = 0.1

        cmd_vel_msg = Twist()
        
        if abs(ang) >= angle_tolerance:
            cmd_vel_msg.angular.z = yaw
            print("cmd_yaw = %.3f" % (yaw))
            self.get_logger().info("ang")
            self.cmd_vel_pub.publish(cmd_vel_msg)

        else :
            if dis >= distance_tolerance:
                cmd_vel_msg.linear.x = x
                cmd_vel_msg.linear.y = y
                print("cmd_x = %.3f, cmd_y = %.3f" % (x, y))
                self.get_logger().info("Goal")
                self.cmd_vel_pub.publish(cmd_vel_msg)

            else :
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.get_logger().info("end")
                self.cmd_vel_pub.publish(cmd_vel_msg)
                quit()
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Goal()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()