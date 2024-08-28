import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='iron_x').find('iron_x')
  world_file_name = 'maze.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',default_value='False',)
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',default_value='true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',default_value='True')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',default_value=world_path)
  
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)

  return ld
