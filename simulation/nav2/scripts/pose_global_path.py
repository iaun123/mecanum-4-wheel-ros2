#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
import math
from tf_transformations import euler_from_quaternion

class PathToCSV(Node):
    def __init__(self):
        super().__init__('path_to_csv')
        self.subscription = self.create_subscription(
            Path,'/plan',self.global_path_callback,10)

    def global_path_callback(self, path_msg):
        path_data = []

        for pose_stamped in path_msg.poses:
            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation

            # Convert quaternion to Euler angles
            quaternion = (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            yaw_degrees = math.degrees(yaw)
            yaw_degrees = (yaw_degrees + 360) % 360  # Normalize to [0, 360) degrees

            # Store the global path waypoints in a list
            path_data.append([f"{position.x:.3f}", f"{position.y:.3f}", f"{yaw_degrees:.3f}"])

        # Save the path data to a CSV file
        with open('global.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['x', 'y', 'yaw'])  # Header
            csvwriter.writerows(path_data)

        self.get_logger().info('Global path saved to global_path.csv')

        # Shutdown the node after saving the path
        exit(0)

def main(args=None):
    rclpy.init(args=args)
    path_to_csv = PathToCSV()
    rclpy.spin(path_to_csv)

if __name__ == '__main__':
    main()
