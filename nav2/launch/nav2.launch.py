from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

    # Bringing our Robot
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('iron_x'),
                                       '/launch','/gazebo.launch.py'])
    ),
    
    #map
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2'),
                                       '/launch','/map.launch.py'])
    ),
    ])