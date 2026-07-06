# ── Imports ───────────────────────────────────────────────────────────────────
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argument references (lazy, resolved at runtime) ─────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization  = LaunchConfiguration('localization')
    robot_ns      = LaunchConfiguration('robot_ns')
    rtabmap_viz   = LaunchConfiguration('rtabmap_viz')

    # ── Declare launch arguments ─────────────────────────────────────────────
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

    # ── Node parameters ──────────────────────────────────────────────────────

    # RTAB-Map specific parameters
    rtabmap_parameters = {
        'subscribe_rgbd':         True,    # subscribe to synchronized RGB-D image
        'subscribe_scan':         True,    # subscribe to the lidar
        'use_action_for_goal':    True,    # use action server for Nav2 goals
        'odom_sensor_sync':       True,    # synchronize odometry with sensors
        'Mem/NotLinkedNodesKept': 'false', # do not keep unlinked nodes in memory
        'Grid/RangeMin':          '0.7',   # ignore lidar points <0.7m (robot body)
        'RGBD/OptimizeMaxError':  '2',     # reject loop closures with high error
        'Grid/DepthMin':          '0.5',   # ignore depth points <0.5m
        'Grid/DepthMax':          '5.0',   # ignore depth points >5m
        'Mem/SaveDepth16Format':  'false', # use float32 depth (meters), not uint16 (mm)
    }

    # Shared parameters for all nodes
    shared_parameters = {
        'frame_id':                      'base_link',  # robot base frame
        'use_sim_time':                  use_sim_time, # Isaac Sim clock
        'odom_frame_id':                 'odom',       # odometry frame
        'tf_tolerance':                  1.0,          # increased TF tolerance for sim
        'wait_for_transform':            1.0,          # increased TF wait for sim
        'Reg/Strategy':                  '1',          # ICP for loop closure
        'Reg/Force3DoF':                 'true',       # 2D SLAM, robot on horizontal plane
        'Mem/NotLinkedNodesKept':        'false',
        'Icp/PointToPlaneMinComplexity': '0.04',       # robust in long corridors
        'Icp/MaxTranslation':            '1',          # reject ICP with translation >1m
        'map_frame_id': 'world',
        'publish_tf':         False, 
    }

    # ── Remappings ────────────────────────────────────────────────────────────
    # Connect RTAB-Map internal topics to Isaac Sim topics
    remappings = [
        ('odom',            '/odom'),        # wheel odometry from Isaac Sim
        ('scan',            '/scan'),        # lidar
        ('rgb/image',       '/rgb'),         # color image
        ('rgb/camera_info', '/camera_info'), # camera calibration
        ('depth/image',     '/depth'),       # depth image
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # Node 1: RGBD Sync
    # Synchronizes RGB + Depth into a single RGBD message
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

    # Node 2: RTAB-Map SLAM
    # Builds the map incrementally while the robot explores
    # Launched only if localization:=false (default mode)
    # -d flag: clears the database on start (new map each session)
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

    # Node 3: RTAB-Map Localization
    # Loads an existing map and localizes the robot within it
    # Does NOT update the map
    # Launched only if localization:=true
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
            'map_frame_id': 'world',
        }],
        remappings=remappings)

    # Node 4: RTAB-Map Viz
    # 3D visualization window for map, graph, and odometry
    # Launched only if rtabmap_viz:=true (default mode)
    node_rtabmap_viz = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=robot_ns,
        output='screen',
        parameters=[rtabmap_parameters, shared_parameters],
        remappings=remappings)

    # ── Assemble everything into LaunchDescription ───────────────────────────
    return LaunchDescription([
        # Arguments first
        arg_use_sim_time,
        arg_localization,
        arg_robot_ns,
        arg_rtabmap_viz,
        # Nodes after
        node_rgbd_sync,
        node_rtabmap_slam,
        node_rtabmap_localization,
        node_rtabmap_viz,
    ])