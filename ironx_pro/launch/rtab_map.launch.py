from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'rtabmap_args': '--delete_db_on_start',
        'frame_id':'base_link',
        'subscribe_depth': True,
        # 'approx_sync':False,
        'ground_truth_frame_id': 'map',
        'ground_truth_base_frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'qos': 1,
        'Vis/CorType': '1',
        'Vis/MaxFeatures': '600',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '80000',
        'publish_map_tf': False,
        }]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
          ('map', 'rtabmap/map'),
          ]

    return LaunchDescription([
        
        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])