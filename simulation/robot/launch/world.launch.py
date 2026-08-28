import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    robot_dir = get_package_share_directory("robot")
    robot_share = os.path.join(get_package_prefix("robot"), "share")
    models_dir = os.path.join(robot_dir, "models")
    
    world_path = os.path.join(robot_dir, 'worlds', 'maze.world')

    gz_resource_paths = f"{robot_share}:{models_dir}:{robot_dir}"
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_paths
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_paths
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time', default_value='true'
    )
    declare_world_cmd = DeclareLaunchArgument(
        name='world', default_value=world_path
    )

    # Start Gazebo Sim
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        declare_use_sim_time_cmd,
        declare_world_cmd,
        start_gazebo_cmd,
        clock_bridge,
    ])

