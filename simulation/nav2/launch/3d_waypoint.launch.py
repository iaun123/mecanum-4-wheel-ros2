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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_pkg = os.path.join(get_package_share_directory('nav2'))
    map_file = os.path.join(nav2_pkg,'maps','map','map.yaml')
    params_file = os.path.join(nav2_pkg,'params','waypoint_3d.yaml')
    rviz_config= os.path.join(nav2_pkg,'rviz','3d.rviz')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('iron_x'),
                                        '/launch','/3d_robot.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),
                                        '/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': params_file}.items(),
        ),
                
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'), 
        
    ])

# ros2 run pcl_ros pcd_to_pointcloud --ros-args 
# -p file_name:=/home/iron_x/iron_x/src/point_cloud_perception/point_cloud_map/map.pcd
# -p tf_frame:=map
