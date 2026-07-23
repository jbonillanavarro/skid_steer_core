import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    skid_steer_nav_dir = get_package_share_directory('skid_steer_nav')

    params_file = os.path.join(skid_steer_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # RTAB-Map ya publica el TF map->odom y el occupancy grid (/map),
        # asi que solo se levanta el stack de navegacion (sin map_server/amcl).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true',
            }.items()
        ),
    ])
