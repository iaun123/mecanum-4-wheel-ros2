import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro

def generate_launch_description():
  robot_name_in_model = 'iron_x'
  gazebo_models_path = 'models'
  robot_localization = 'config/ekf.yaml'
  rviz_config_file_path = 'rviz/basic.rviz'
  urdf_file_path = 'urdf/gazebo.urdf'
  world_file_path = 'worlds/home.world'
  
  # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.036711'
  spawn_yaw_val = '0.0'
  
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='iron_x').find('iron_x')
  default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
  robot_localization_file_path = os.path.join(pkg_share, robot_localization) 
  default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
  world_path = os.path.join(pkg_share, world_file_path)
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',default_value='')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',default_value='false')
        
  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', default_value='true')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',default_value=default_rviz_config_path)

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',default_value='False')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', default_value=default_urdf_model_path)
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',default_value='True')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',default_value='True')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',default_value='true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',default_value='True')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',default_value=world_path)
  
  robot_description = xacro.process_file(default_urdf_model_path)
   
  # Specify the actions
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
                'robot_description': robot_description.toxml()}],
    arguments=[default_urdf_model_path])

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Launch the robot  
  spawn_entity_cmd = Node(
    package="gazebo_ros", 
    executable="spawn_entity.py",
    arguments=["-entity",robot_name_in_model,
              "-topic", "robot_description",
              "-x", spawn_x_val,
              "-y", spawn_y_val,
              "-z", spawn_z_val,
              "-Y", spawn_yaw_val,
          ],
    output="screen",
    )

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])    


  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_rviz_cmd)

  return ld