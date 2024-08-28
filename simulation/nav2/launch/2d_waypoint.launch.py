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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_pkg = os.path.join(get_package_share_directory('nav2'))
    map_file = os.path.join(nav2_pkg,'maps','map','map.yaml')
    params_file = os.path.join(nav2_pkg,'params','waypoint_2d.yaml')
    rviz_config= os.path.join(nav2_pkg,'rviz','test.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('iron_x'),
                                        '/launch','/2d_robot.launch.py'])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), #nav2_bringup
                                        '/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': params_file,
            }.items(),
        ),   
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
