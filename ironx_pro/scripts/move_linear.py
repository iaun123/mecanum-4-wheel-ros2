#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math

class Goal(Node):
    def __init__(self):
        super().__init__("move_linear")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Odometry()
        self.up_x = 0.0
        self.up_y = 0.0
    
    def cal_error(self, goal_x, goal_y):
        vel_x = 3.0
        vel_y = 3.0

        dis_x = goal_x - self.up_x
        dis_y = goal_y - self.up_y
        distance = math.sqrt(math.pow(dis_x, 2) + math.pow(dis_y, 2))

        linearx_input = vel_x * dis_x  # m/s
        lineary_input = vel_y * dis_y  # m/s
    
        return linearx_input, lineary_input, distance
    
    def pose_callback(self, data):
        self.pose = data
        self.up_x = self.pose.pose.pose.position.x
        self.up_y = self.pose.pose.pose.position.y

    def go_to_goal(self):
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])

        cmd_linearx, cmd_lineary, dis = self.cal_error(goal_x, goal_y)

        print('')
        print("Displacement: linearx = %.3f, lineary = %.3f" % (cmd_linearx, cmd_lineary))
        print("Distance to goal = %.3f" % dis)
        
        max_linearx = 0.8
        max_lineary = 0.8
        # Limit x velocity
        if cmd_linearx > max_linearx: 
            cmd_linearx = max_linearx
        elif cmd_linearx < -max_linearx: 
            cmd_linearx = -max_linearx
        # Limit y velocity
        if cmd_lineary > max_lineary: 
            cmd_lineary = max_lineary
        elif cmd_lineary < -max_lineary: 
            cmd_lineary = -max_lineary

        distance_tolerance = 0.01
        cmd_vel_msg = Twist()
        
        if dis >= distance_tolerance:
            cmd_vel_msg.linear.x = cmd_linearx
            cmd_vel_msg.linear.y = cmd_lineary
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info("Moving towards goal")
        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)  # Ensure stop command is published before exiting
            self.get_logger().info("Reached goal, stopping")
            # self.timer.cancel()  # Stop the timer to stop calling go_to_goal
            # rclpy.shutdown()
            quit()  

        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot stops by sending zero velocities
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
