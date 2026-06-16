from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():

    map_dir = os.path.expanduser('~/unitree_ros2/map')
    os.makedirs(map_dir, exist_ok=True)

    return LaunchDescription([

        # ============================================================
        # 1. TF STATIQUES
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            output='screen',
            arguments=['0', '0', '0.3', '3.14159', '0', '0',
                       'base_link', 'utlidar_lidar']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            output='screen',
            arguments=['0', '0', '0.3', '3.14159', '0', '0',
                       'base_link', 'utlidar_imu']
        ),

        # ============================================================
        # 2. RTABMAP ODOMETRIE LIO
        #    Fusionne LiDAR 3D + IMU à haute résolution
        # ============================================================
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                # Frames
                'frame_id':                  'base_link',
                'odom_frame_id':             'odom',

                # Sync LiDAR + IMU
                'approx_sync':               True,
                'approx_sync_max_interval':  0.05,
                'wait_for_transform':        0.3,
                'subscribe_scan_cloud':      True,

                # Fusion IMU
                'guess_from_tf':             False,
                'odom_sensor_sync':          True,

                # ICP Haute Précision (Voxel à 3cm pour l'odométrie)
                'Icp/VoxelSize':             '0.03',
                'Icp/MaxCorrespondenceDistance': '0.3',
                'Icp/PointToPlane':          'true',
                'Icp/Iterations':            '80',
                'Icp/OutlierRatio':          '0.85',
                'Icp/CorrespondenceRatio':   '0.7',

                # Filtre odométrie
                'Odom/Strategy':             '0',
                'Odom/GuessMotion':          'true',
                'Odom/ResetCountdown':       '0',

                'OdomF2M/MaxSize':           '10000',
                'OdomF2M/ScanSubtraction':   '0.0',

                'use_sim_time':              False,
            }],
            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                ('imu',        '/utlidar/imu'),
            ]
        ),

        # ============================================================
        # 3. RTAB-MAP SLAM LIO PURE
        #    Construit la carte 3D à partir de l'odométrie ICP
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    parameters=[{

                        # Abonnements : Uniquement le LiDAR 3D
                        'subscribe_rgb':            False,
                        'subscribe_depth':          False,
                        'subscribe_rgbd':           False,
                        'subscribe_stereo':         False,
                        'subscribe_scan_cloud':     True,

                        # Frames
                        'frame_id':                 'base_link',
                        'odom_frame_id':            'odom',
                        'map_frame_id':             'map',

                        # Sync globale
                        'approx_sync':              True,
                        'approx_sync_max_interval': 0.1,
                        'wait_for_transform':       0.3,

                        # Sauvegarde de la base de données (.db)
                        'database_path':            map_dir + '/rtabmap_lio_pure.db',
                        'Mem/SaveDepthImages':      'false',
                        'Mem/NotLinkedNodesKept':   'true',
                        'Mem/SaveClouds':           'true',
                        'Mem/CompressionParallelized': 'false',

                        # ICP SLAM (Alignement géométrique de précision - Voxel 2cm)
                        'Reg/Strategy':             '1',   # 1 = ICP pur (Géométrique)
                        'Icp/VoxelSize':            '0.02',
                        'Icp/MaxCorrespondenceDistance': '0.1',
                        'Icp/PointToPlane':         'true',
                        'Icp/Iterations':           '30',
                        'Icp/OutlierRatio':         '0.85',
                        'Icp/CorrespondenceRatio':  '0.7',

                        # DEBRIDAGE VERTICAL : On désactive les limites pour voir le toit !
                        'Grid/FromDepth':           'false',
                        'Grid/RayTracing':          'true',
                        'Grid/3D':                  'true',
                        'Grid/MaxObstacleHeight':   '0.0',  # 0.0 supprime le filtre de hauteur max
                        'Grid/MinGroundHeight':     '0.0',  # 0.0 supprime le filtre de hauteur min
                        'Grid/NormalsSegmentation': 'true',
                        'Grid/MaxGroundAngle':      '45',
                        'Grid/ClusterRadius':       '0.1',
                        'Grid/MinClusterSize':      '10',

                        # Nuage de points dense centimétrique (Objets très nets)
                        'cloud_decimation':         '1',    # Garder tous les points LiDAR
                        'cloud_max_depth':          '30.0', # Portée maximale utile en intérieur
                        'cloud_min_depth':          '0.2',
                        'cloud_voxel_size':         '0.0', # Résolution à 1 cm (Haute Définition)
                        'cloud_noise_filtering_radius':        '0.03',
                        'cloud_noise_filtering_min_neighbors': '3',

                        # Gestion mémoire SLAM
                        'Mem/IncrementalMemory':    'true',
                        'Mem/InitWMWithAllNodes':   'false',
                        'Mem/STMSize':              '80',
                        'Mem/RehearsalSimilarity':  '0.45',

                        # Fermetures de boucles par proximité spatiale (LiDAR Loop Closure)
                        'RGBD/LoopClosureEnabled':              'true',
                        'RGBD/ProximityBySpace':                'true',
                        'RGBD/ProximityMaxGraphDepth':          '0',
                        'RGBD/ProximityPathMaxNeighbors':       '5',
                        'RGBD/ProximityAngle':                  '45',
                        'RGBD/ProximityPathFilteringRadius':    '0.5',
                        'RGBD/LinearUpdate':                    '0.005',
                        'RGBD/AngularUpdate':                   '0.002',

                        # Optimisation du graphe de pose
                        'Optimizer/Strategy':       '1', # G2O
                        'Optimizer/Iterations':     '100',
                        'Optimizer/Epsilon':        '0.00001',
                        'Optimizer/Robust':         'true',

                        'Rtabmap/CreateGrid':       'true',
                        'Rtabmap/TimeThr':          '0',

                        'use_sim_time':             False,
                    }],
                    remappings=[
                        ('scan_cloud',      '/utlidar/cloud_deskewed'),
                        ('odom',            '/icp_odom'),
                    ]
                ),
            ]
        ),

        # ============================================================
        # 4. RTAB-MAP VISUALIZATION
        # ============================================================
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    name='rtabmap_viz',
                    output='screen',
                    parameters=[{
                        'subscribe_scan_cloud':     True,
                        'subscribe_rgb':            False,
                        'frame_id':                 'base_link',
                        'odom_frame_id':            'odom',
                        'approx_sync':              True,
                        'cloud_decimation':         '1',
                        'cloud_max_depth':          '15.0',
                        'cloud_voxel_size':         '0.02',
                        'use_sim_time':             False,
                    }],
                    remappings=[
                        ('scan_cloud',      '/utlidar/cloud_deskewed'),
                        ('odom',            '/icp_odom'),
                    ]
                ),
            ]
        ),

        # ============================================================
        # 5. RVIZ2 (Pour l'analyse sur ton PC portable)
        # ============================================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                ),
            ]
        ),
    ])