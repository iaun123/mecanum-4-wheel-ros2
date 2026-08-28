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
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    robot_pkg = get_package_share_directory('robot')

    # Default to my_map if exists, otherwise 12x12
    my_map_path = os.path.join(nav2_pkg, 'maps', 'my_map', 'my_map.yaml')
    default_map_file = my_map_path if os.path.exists(my_map_path) else os.path.join(nav2_pkg, 'maps', '12x12', '12x12.yaml')
    
    default_params_file = os.path.join(nav2_pkg, 'params', '2d.yaml')
    rviz_config_file = os.path.join(nav2_pkg, 'rviz', '2d.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    sim = LaunchConfiguration('sim')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_map = DeclareLaunchArgument('map', default_value=default_map_file, description='Full path to map yaml file')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Nav2 params')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true', description='Autostart nav2 stack')
    declare_sim = DeclareLaunchArgument('sim', default_value='false', description='Launch simulation alongside Nav2')

    # Simulation (Optional)
    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'robot.launch.py')),
        condition=IfCondition(sim)
    )

    # Nav2 Bringup (use_composition=False for maximum stability)
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'True',
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
        declare_map,
        declare_params,
        declare_autostart,
        declare_sim,
        start_sim,
        start_nav2,
        rviz_node,
    ])
