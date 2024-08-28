import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    iron_x_dir = get_package_share_directory("iron_x")
    iron_x_share = os.path.join(get_package_prefix("iron_x"), "share")
    robot_name_in_model = 'pro'
    model_file = os.path.join(iron_x_dir, "urdf", "ironx_pro.urdf")
    world_file = os.path.join(iron_x_dir, "worlds", "home.world")
    rviz_file = os.path.join(iron_x_dir, "rviz", "basic.rviz")
    
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val =  '0.006473' # 12x12 = 0.106450 home = 0.006473
    spawn_yaw_val = '0.0'

    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='true')
    declare_world_cmd = DeclareLaunchArgument(name='world',default_value=world_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
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
        launch_arguments={"world": world}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py"))
    )

    spawn_robot = Node(package="gazebo_ros", 
                       executable="spawn_entity.py",
                       arguments=["-entity", robot_name_in_model,
                                  "-topic", "robot_description",
                                  "-x", spawn_x_val,
                                  "-y", spawn_y_val,
                                  "-z", spawn_z_val,
                                  "-Y", spawn_yaw_val,
                                ],
                       output="screen",
    )   

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='localization', #ekf_filter_node
       output='screen',
       parameters=[os.path.join(iron_x_share, 'config/2d_ekf.yaml'), 
                   {'use_sim_time': use_sim_time}],
    #    remappings=[("/odometry/filtered", "/odom/localization")]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_cmd,

        env_var,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        robot_localization_node,
        # rviz_node,
    ])