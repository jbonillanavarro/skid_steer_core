from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # ---------------------------------------------------------
        # 1. LA CAJA SINCRONIZADORA (rgbd_sync)
        # ---------------------------------------------------------
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True, # Ponlo en True por si Isaac Sim tiene un microsegundo de lag entre RGB y Depth
                'use_sim_time': True,
            }],
            remappings=[
                # (Lo que RTAB-Map pide , Lo que Isaac Sim publica)
                ('rgb/image', '/rgb'),
                ('depth/image', '/depth'),
                ('rgb/camera_info', '/camera_info')
            ]
            # Nota: Este nodo generará automáticamente el topic '/rgbd_image'
        ),

        # ---------------------------------------------------------
        # 2. LA CAJA CEREBRO (rtabmap)
        # ---------------------------------------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            # El argumento '-d' borra la base de datos del mapa anterior al arrancar.
            # ¡Súper útil mientras haces pruebas para no mezclar mapas viejos!
            arguments=['-d'], 
            parameters=[{
                'frame_id': 'base_link',   # El eslabón central de tu robot
                'subscribe_depth': False,  # Ya no usamos depth suelto...
                'subscribe_rgbd': True,    # ...usamos el empaquetado de rgbd_sync
                'subscribe_odom_info': False, # Porque usamos tu odometría estándar
                'use_sim_time': True,
            }],
            remappings=[
                # (Lo que RTAB-Map pide , Lo que Isaac Sim publica/Genera)
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
            # Traslación a CERO. 
            # Rotación: Yaw(Z=0º), Pitch(Y=180º), Roll(X=180º)
            arguments=[
                '0.0', '0.0', '0.0',          
                '0.0', '0.0', '3.1416',   # El valor mágico de 180º es 3.1416
                'bumblebee_stereo_left_frame', 
                'bumblebee_stereo_left_optical_frame'
            ]
        )
    ])