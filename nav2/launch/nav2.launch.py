from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    nav2_pkg = os.path.join(get_package_share_directory('nav2'))
    map_file = os.path.join(nav2_pkg,'maps','map','maps.yaml')
    params_file = os.path.join(nav2_pkg,'params','2d.yaml')
    rviz_config= os.path.join(nav2_pkg,'rviz','2d.rviz')

    return LaunchDescription([
    # Bringing our Robot
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('iron_x'),
                                       '/launch','/robot.launch.py'])
    ),
    
    # Integerating Nav2 Stack
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),
                                       '/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file,
        }.items(),
    ),

    # Rviz2 bringup
    Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config]
    ),

        ])