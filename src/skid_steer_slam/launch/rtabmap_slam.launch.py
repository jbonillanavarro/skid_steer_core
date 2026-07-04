# ─────────────────────────────────────────────────────────────────────────────
# Skid-Steer Robot SLAM Launch File
# ─────────────────────────────────────────────────────────────────────────────
#
# Requirements:
#   Isaac Sim running with Jackal robot publishing:
#       /scan                        (sensor_msgs/LaserScan)
#       /rgb                         (sensor_msgs/Image)
#       /camera_info                 (sensor_msgs/CameraInfo)
#       /depth                       (sensor_msgs/Image)
#       /odom                        (nav_msgs/Odometry)
#       /tf                          (world → odom → base_link → sensors)
#       /clock                       (rosgraph_msgs/Clock)
#
# Usage:
#   SLAM mode (default):
#     $ ros2 launch ss_slam skid_steer_slam.launch.py
#
#   Localization mode (map already built):
#     $ ros2 launch ss_slam skid_steer_slam.launch.py localization:=true
#
#   Without visualization:
#     $ ros2 launch ss_slam skid_steer_slam.launch.py rtabmap_viz:=false
#
#   Show all available arguments:
#     $ ros2 launch ss_slam skid_steer_slam.launch.py --show-args
#
# ─────────────────────────────────────────────────────────────────────────────

# ── Imports ───────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Referencias a los argumentos (lazy, se resuelven al ejecutar) ─────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization  = LaunchConfiguration('localization')
    robot_ns      = LaunchConfiguration('robot_ns')
    rtabmap_viz   = LaunchConfiguration('rtabmap_viz')

    # ── Declaración de argumentos ─────────────────────────────────────────────
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use Isaac Sim clock. Always true when running in simulation.')

    arg_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        choices=['true', 'false'],
        description='Launch rtabmap in localization mode. '
                    'A map must have been built previously (rtabmap.db exists).')

    arg_robot_ns = DeclareLaunchArgument(
        'robot_ns',
        default_value='jackal',
        description='Robot namespace. Prefixes all topics and node names. '
                    'Change if using a different skid-steer robot.')

    arg_rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        choices=['true', 'false'],
        description='Start rtabmap_viz visualization window.')

    # ── Parámetros de los nodos ───────────────────────────────────────────────

    # Parámetros específicos de RTAB-Map
    rtabmap_parameters = {
        'subscribe_rgbd':         True,    # suscribirse a imagen RGB-D sincronizada
        'subscribe_scan':         True,    # suscribirse al lidar
        'use_action_for_goal':    True,    # usar action server para goals de Nav2
        'odom_sensor_sync':       True,    # sincronizar odometría con sensores
        'Mem/NotLinkedNodesKept': 'false', # no guardar nodos sin conexión en memoria
        'Grid/RangeMin':          '0.7',   # ignorar puntos del lidar a <0.7m (cuerpo del robot)
        'RGBD/OptimizeMaxError':  '2',     # rechazar loop closures con error alto
        'Grid/DepthMin':          '0.5',   # ignorar puntos de profundidad a <0.5m
        'Grid/DepthMax':          '5.0',   # ignorar puntos de profundidad a >5m
        'Mem/SaveDepth16Format':  'false', # depth en float32 (metros), no uint16 (mm)
    }

    # Parámetros compartidos entre todos los nodos
    shared_parameters = {
        'frame_id':                      'base_link',  # frame base del robot
        'use_sim_time':                  use_sim_time, # reloj de Isaac Sim
        'odom_frame_id':                 'odom',       # frame de odometría
        'tf_tolerance':                  1.0,          # tolerancia TF aumentada para sim
        'wait_for_transform':            1.0,          # espera TF aumentada para sim
        'Reg/Strategy':                  '1',          # ICP para loop closure
        'Reg/Force3DoF':                 'true',       # SLAM 2D, robot en plano horizontal
        'Mem/NotLinkedNodesKept':        'false',
        'Icp/PointToPlaneMinComplexity': '0.04',       # robusto en pasillos largos
        'Icp/MaxTranslation':            '1',          # rechazar ICP con traslación >1m
    }

    # ── Remappings ────────────────────────────────────────────────────────────
    # Conecta los topics internos de RTAB-Map con los topics reales de Isaac Sim
    remappings = [
        ('odom',            '/odom'),        # wheel odometry de Isaac Sim
        ('scan',            '/scan'),        # lidar
        ('rgb/image',       '/rgb'),         # imagen de color
        ('rgb/camera_info', '/camera_info'), # calibración de la cámara
        ('depth/image',     '/depth'),       # imagen de profundidad
    ]

    # ── Nodos ─────────────────────────────────────────────────────────────────

    # Nodo 1: RGBD Sync
    # Sincroniza RGB + Depth en un único mensaje RGBD
    node_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=robot_ns,
        output='screen',
        parameters=[{
            'approx_sync':  True,
            'use_sim_time': use_sim_time,
        }],
        remappings=remappings)

    # Nodo 2: RTAB-Map SLAM
    # Construye el mapa incrementalmente mientras el robot explora
    # Solo se lanza si localization:=false (modo por defecto)
    # -d flag: borra la base de datos al inicio (mapa nuevo cada sesión)
    node_rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=robot_ns,
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings,
        arguments=['-d'])

    # Nodo 3: RTAB-Map Localización
    # Carga el mapa ya construido y localiza el robot en él
    # NO actualiza el mapa
    # Solo se lanza si localization:=true
    node_rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=robot_ns,
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters, {
            'Mem/IncrementalMemory':  'False', # no añade nodos nuevos al mapa
            'Mem/InitWMWithAllNodes': 'True',  # carga el mapa completo en memoria
        }],
        remappings=remappings)

    # Nodo 4: RTAB-Map Viz
    # Ventana de visualización 3D del mapa, grafo y odometría
    # Solo se lanza si rtabmap_viz:=true (modo por defecto)
    node_rtabmap_viz = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=robot_ns,
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings)

    # ── Ensamblar todo en LaunchDescription ───────────────────────────────────
    return LaunchDescription([
        # Argumentos primero
        arg_use_sim_time,
        arg_localization,
        arg_robot_ns,
        arg_rtabmap_viz,
        # Nodos después
        node_rgbd_sync,
        node_rtabmap_slam,
        node_rtabmap_localization,
        node_rtabmap_viz,
    ])