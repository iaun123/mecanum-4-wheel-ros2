import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration('map',
        default=os.path.join(get_package_share_directory('ironx_pro'),'map','mapshop','mapshop.yaml'))
    
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    pcd_params_file = LaunchConfiguration('params_file',default=os.path.join(
        get_package_share_directory('ironx_pro'), 'params', '3d_waypoint.yaml'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('ironx_pro'),'rviz','3d.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value=map_dir,
            description='Navigation map path'),
                    
        DeclareLaunchArgument(
            name='use_pcd',
            default_value='False',
            description='Default is use only LaserScan data use_pcd:=True to active the integration of point cloud data to detect the obstacles in the navigation.'),
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': pcd_params_file
            }.items(), 
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

    ])

# ros2 run pcl_ros pcd_to_pointcloud --ros-args 
# -p file_name:=/home/iron_x/test/src/ironx_pro/map/mapshop/mapshop.pcd
# -p tf_frame:=map