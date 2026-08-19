from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.02,
                'use_sim_time': True,
            }],
            remappings=[
                ('rgb/image', '/rgb'),
                ('depth/image', '/depth'),
                ('rgb/camera_info', '/camera_info')
            ]
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'], 
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'subscribe_odom_info': False,
                'use_sim_time': True,
                'approx_sync_max_interval': 0.02,
                'cloud_max_depth': 20.0,
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/RangeMax': '6.0',
                'GridGlobal/ProbClampingMax': '0.8',   # por defecto 0.971
                'GridGlobal/ProbMiss': '0.3',          # por defecto 0.4
                'Grid/RayTracing': 'true',
                'Rtabmap/DetectionRate': '2.0',
                'Reg/Force3DoF': 'true',
                'Optimizer/Slam2D': 'true',
                'Grid/NoiseFilteringRadius': '0.2',
                'Grid/DepthDecimation': '2',
                'Grid/NoiseFilteringMinNeighbors': '10',
                'Grid/MinClusterSize': '25',
                'GridGlobal/Eroded': 'true',
                'qos_overrides./map.publisher.durability': 'transient_local',
                'qos_overrides./map.publisher.reliability': 'reliable',
            }],
            remappings=[
                ('odom', '/odom'),
                ('rgbd_image', '/rgbd_image')
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('odom', '/odom'),
                ('rgbd_image', '/rgbd_image')
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '0.0', '0.0', '0.0',          
                '0.0', '0.0', '3.1416',
                'bumblebee_stereo_left_frame', 
                'bumblebee_stereo_left_optical_frame'
            ]
        )
    ])