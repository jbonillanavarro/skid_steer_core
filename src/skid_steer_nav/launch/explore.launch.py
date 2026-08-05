import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    skid_steer_nav_dir = get_package_share_directory('skid_steer_nav')
    params_file = os.path.join(skid_steer_nav_dir, 'config', 'explore_params.yaml')

    return LaunchDescription([
        # Explora fronteras entre lo conocido y lo desconocido en el global_costmap
        # y manda goals a Nav2 (navigate_to_pose) automaticamente hasta agotarlas.
        Node(
            package='explore_lite',
            name='explore_node',
            executable='explore',
            parameters=[params_file],
            output='screen',
            arguments=[
                '--ros-args',
                '--log-level', 'explore_node:=debug',
                # el logger de rclcpp_action es hijo de explore_node y hereda
                # su nivel, llenando la salida de "Received status for unknown
                # goal" cada vez que llega el estado de un goal ya preemptado
                '--log-level', 'explore_node.rclcpp_action:=info',
            ],
        ),
    ])
