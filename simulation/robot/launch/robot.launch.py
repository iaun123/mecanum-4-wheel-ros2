import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    robot_dir = get_package_share_directory("robot")
    robot_share = os.path.join(get_package_prefix("robot"), "share")
    models_dir = os.path.join(robot_dir, "models")
    
    robot_name_in_model = 'pro'
    model_file = os.path.join(robot_dir, "urdf", "robot.urdf")
    world_file = os.path.join(robot_dir, "worlds", "12x12.world")
    rviz_file = os.path.join(robot_dir, "rviz", "basic.rviz")
    
    # Environment variables for Gazebo Sim to find models and meshes
    gz_resource_paths = f"{robot_share}:{models_dir}:{robot_dir}"
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_paths
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_paths
    )

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.106450'
    spawn_yaw_val = '0.0'

    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='true')
    declare_world_cmd = DeclareLaunchArgument(name='world', default_value=world_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    robot_description_content = Command(['xacro ', model_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # Launch Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ['-r ', world]
        }.items()
    )

    # Spawn robot in Gazebo Sim
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name_in_model,
            "-topic", "robot_description",
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
            "-Y", spawn_yaw_val,
            "-allow_renaming", "true"
        ],
        output="screen",
    )

    # Bridge between Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(robot_dir, 'config', '2d_ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        declare_use_sim_time,
        declare_world_cmd,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_robot,
        bridge,
        robot_localization_node,
        rviz_node,
    ])