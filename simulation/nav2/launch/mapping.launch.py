import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2')
    robot_pkg = get_package_share_directory('robot')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(nav2_pkg, 'params', 'slam_toolbox.yaml')
    rviz_config_file = os.path.join(nav2_pkg, 'rviz', 'test.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    declare_sim = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Launch Gazebo simulation along with SLAM'
    )

    # Simulation (Optional)
    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'robot.launch.py')),
        condition=IfCondition(sim)
    )

    # SLAM Toolbox Node
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_sim,
        start_sim,
        start_slam_toolbox,
        rviz_node,
    ])
