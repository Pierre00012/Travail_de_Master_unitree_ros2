from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Dossier de sauvegarde de la carte
    map_dir = os.path.expanduser('~/unitree_ros2/map')
    os.makedirs(map_dir, exist_ok=True)

    return LaunchDescription([

        # ============================================
        # 1. TF STATIQUE : base_link → utlidar_lidar
        #    Le LiDAR est monté sur le robot a ~30cm
        #    et inversé sur l'axe X (rotation 180°)
        # ============================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            output='screen',
            arguments=[
                '0',          # x
                '0',          # y
                '0.3',        # z — hauteur LiDAR sur le robot
                '3.14159',    # roll — rotation 180° car LiDAR inversé
                '0',          # pitch
                '0',          # yaw
                'base_link',  # frame parent
                'utlidar_lidar'  # frame enfant
            ]
        ),

        # ============================================
        # 2. RTAB-Map SLAM
        #    Entrées : LiDAR 3D + odométrie
        #    Sortie  : carte 3D + carte 2D + poses
        # ============================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{

                # ── Désactive RGB-D — LiDAR uniquement ──
                'subscribe_rgb':          False,
                'subscribe_depth':        False,
                'subscribe_rgbd':         False,
                'subscribe_stereo':       False,
                'subscribe_scan_cloud':   True,

                # ── Frames ──
                'frame_id':               'base_link',
                'odom_frame_id':          'odom',
                'map_frame_id':           'map',

                # ── Sync LiDAR 14Hz / odom 149Hz ──
                'approx_sync':            True,
                'approx_sync_max_interval': 0.1,
                'wait_for_transform':     0.2,

                # ── Sauvegarde ──
                'database_path':          map_dir + '/rtabmap.db',
                'Mem/SaveDepthImages':    'true',
                'Mem/NotLinkedNodesKept': 'true',

                # ── ICP pour LiDAR 3D ──
                'Reg/Strategy':           '1',      # ICP
                'Icp/VoxelSize':          '0.1',
                'Icp/MaxCorrespondenceDistance': '0.2',
                'Icp/PointToPlane':       'true',
                'Icp/Iterations':         '30',
                'Icp/MaxTranslation':     '0.5',
                'Icp/MaxRotation':        '0.5',

                # ── Grille 2D occupancy ──
                'Grid/FromDepth':         'false',
                'Grid/RayTracing':        'true',
                'Grid/CellSize':          '0.05',
                'Grid/3D':                'false',
                'Grid/UnknownSpaceFilled': 'false',
                'Grid/MaxObstacleHeight': '0.6',
                'Grid/MinGroundHeight':   '-0.1',

                # ── SLAM incrémental ──
                'Mem/IncrementalMemory':  'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/STMSize':            '30',

                # ── Détection de boucles ──
                'RGBD/LoopClosureEnabled': 'true',
                'RGBD/ProximityBySpace':   'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',

                # ── Nuage de points 3D en sortie ──
                'RGBD/LinearUpdate':      '0.1',
                'RGBD/AngularUpdate':     '0.05',

                'use_sim_time':           False,
            }],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('odom',       '/utlidar/robot_odom'),
            ]
        ),

        # ============================================
        # 3. RTAB-Map Visualization
        # ============================================
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_scan_cloud':   True,
                'subscribe_rgb':          False,
                'subscribe_depth':        False,
                'frame_id':               'base_link',
                'odom_frame_id':          'odom',
                'approx_sync':            True,
                'use_sim_time':           False,
            }],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('odom',       '/utlidar/robot_odom'),
            ]
        ),
    ])