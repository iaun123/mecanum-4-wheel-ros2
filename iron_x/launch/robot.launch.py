import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
import xacro
import launch_ros

def generate_launch_description():
    iron_x_dir = get_package_share_directory("iron_x")
    iron_x_share = os.path.join(get_package_prefix("iron_x"), "share")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    model_file = os.path.join(iron_x_dir, "urdf", "gazebo.urdf")
    rviz_file = os.path.join(iron_x_dir, "rviz", "basic.rviz")
    world_file = os.path.join(iron_x_dir, "worlds", "stage.world")

    # Pose where we want to spawn the robot
    spawn_x_val = '-2.2'
    spawn_y_val = '0.08'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    declare_use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='True')
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false')
    
    use_rviz =  LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = xacro.process_file(model_file)
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", iron_x_share)
   
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time" : use_sim_time,
                     "robot_description": robot_description.toxml()}],
        arguments=[model_file]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py"),),
        launch_arguments={"world": world_file}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py"))
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                       arguments=["-entity", "iron_x",
                                  "-topic", "robot_description",
                                  "-x", spawn_x_val,
                                  "-y", spawn_y_val,
                                  "-z", spawn_z_val,
                                  "-Y", spawn_yaw_val,
                                ],
                       output="screen",
    )   
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='localization', #ekf_filter_node
       output='screen',
       parameters=[os.path.join(iron_x_share, 'config/ekf.yaml'), 
                   {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_use_sim_time,

        env_var,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        rviz_node,
        robot_localization_node,
    ])