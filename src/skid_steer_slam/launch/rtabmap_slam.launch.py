from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'camera_frame_id': 'bumblebee_stereo_left_frame',
                'tf_tolerance': 0.5,
                'wait_for_transform': 0.5,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': True,
                'subscribe_odom': True,
                'use_sim_time': True,
                'Grid/Sensor': '1',
                'Reg/Strategy': '1',
                'RGBD/NeighborLinkRefining': '10',
                'RGBD/ProximityPathMaxNeighbors': '10'
            }],
            remappings=[
                ('rgb/image', '/rgb'),
                ('depth/image', '/depth'),
                ('rgb/camera_info', '/camera_info'),
                ('scan', '/scan'),
                ('odom', '/odom')
            ],
            arguments=['-d']
        )
    ])