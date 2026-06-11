from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    use_camera = LaunchConfiguration('use_camera')
    map_dir = os.path.expanduser('~/unitree_ros2/map')
    os.makedirs(map_dir, exist_ok=True)

    return LaunchDescription([

        # ============================================================
        # ARGUMENTS
        # ============================================================
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            description='Enable RGB camera input for RTAB-Map'
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Depth image topic if available'
        ),
        DeclareLaunchArgument(
            'scan_cloud_topic',
            default_value='/utlidar/cloud_deskewed',
            description='LiDAR point cloud topic'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/utlidar/robot_odom',
            description='Odometry topic'
        ),

        # ============================================================
        # TF STATIQUE base_link → utlidar_lidar
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            output='screen',
            arguments=[
                '0', '0', '0.3',
                '3.14159', '0', '0',
                'base_link',
                'utlidar_lidar'
            ]
        ),

        # ============================================================
        # RTAB-MAP SLAM
        # ============================================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{

                # Abonnements
                'subscribe_rgb':            use_camera,
                'subscribe_depth':          False,
                'subscribe_rgbd':           False,
                'subscribe_stereo':         False,
                'subscribe_scan_cloud':     True,

                # Frames
                'frame_id':                 'base_link',
                'odom_frame_id':            'odom',
                'map_frame_id':             'map',

                # Sync LiDAR 14Hz / odom 149Hz
                'approx_sync':              True,
                'approx_sync_max_interval': 0.08,
                'wait_for_transform':       0.3,

                # Sauvegarde — force les nuages dans la DB
                'database_path':            map_dir + '/rtabmap.db',
                'Mem/SaveDepthImages':      'true',
                'Mem/NotLinkedNodesKept':   'true',
                'Mem/SaveClouds':           'true',

                # ICP LiDAR précision maximale
                'Reg/Strategy':             '1',
                'Icp/VoxelSize':            '0.03',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Icp/PointToPlane':         'true',
                'Icp/Iterations':           '60',
                'Icp/MaxTranslation':       '0.3',
                'Icp/MaxRotation':          '0.3',
                'Icp/OutlierRatio':         '0.85',
                'Icp/CorrespondenceRatio':  '0.7',

                # Grille 3D haute résolution
                'Grid/FromDepth':           'false',
                'Grid/RayTracing':          'true',
                'Grid/CellSize':            '0.03',
                'Grid/3D':                  'true',
                'Grid/UnknownSpaceFilled':  'false',
                'Grid/MaxObstacleHeight':   '2.0',
                'Grid/MinGroundHeight':     '-0.2',
                'Grid/NormalsSegmentation': 'true',
                'Grid/MaxGroundAngle':      '45',
                'Grid/ClusterRadius':       '0.1',
                'Grid/MinClusterSize':      '10',

                # Nuage dense en sortie
                'cloud_decimation':         '1',
                'cloud_max_depth':          '10.0',
                'cloud_min_depth':          '0.1',
                'cloud_voxel_size':         '0.03',
                'cloud_noise_filtering_radius':         '0.05',
                'cloud_noise_filtering_min_neighbors':  '5',

                # SLAM
                'Mem/IncrementalMemory':    'true',
                'Mem/InitWMWithAllNodes':   'false',
                'Mem/STMSize':              '50',
                'Mem/RehearsalSimilarity':  '0.45',

                # Fermeture de boucles
                'RGBD/LoopClosureEnabled':              'true',
                'RGBD/ProximityBySpace':                'true',
                'RGBD/ProximityMaxGraphDepth':          '0',
                'RGBD/ProximityPathMaxNeighbors':       '5',
                'RGBD/ProximityAngle':                  '45',
                'RGBD/ProximityPathFilteringRadius':    '0.5',

                # Mise à jour très fréquente (robot lent)
                'RGBD/LinearUpdate':        '0.02',
                'RGBD/AngularUpdate':       '0.01',

                # Optimisation graphe de poses
                'Optimizer/Strategy':       '1',
                'Optimizer/Iterations':     '100',
                'Optimizer/Epsilon':        '0.00001',
                'Optimizer/Robust':         'true',

                # Détection boucles LiDAR
                'LccIcp/Type':              '1',
                'LccIcp/VoxelSize':         '0.05',
                'LccIcp/MaxDistance':       '0.1',

                'Rtabmap/CreateGrid':       'true',
                'Rtabmap/TimeThr':          '0',

                'use_sim_time':             False,
            }],
            remappings=[
                ('scan_cloud',      LaunchConfiguration('scan_cloud_topic')),
                ('odom',            LaunchConfiguration('odom_topic')),
                ('rgb/image',       LaunchConfiguration('rgb_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('depth/image',     LaunchConfiguration('depth_topic')),
            ]
        ),

        # ============================================================
        # RTAB-MAP VISUALIZATION
        # ============================================================
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_scan_cloud':     True,
                'subscribe_rgb':            use_camera,
                'subscribe_depth':          False,
                'frame_id':                 'base_link',
                'odom_frame_id':            'odom',
                'approx_sync':              True,
                'cloud_decimation':         '1',
                'cloud_max_depth':          '10.0',
                'cloud_voxel_size':         '0.03',
                'use_sim_time':             False,
            }],
            remappings=[
                ('scan_cloud',      LaunchConfiguration('scan_cloud_topic')),
                ('odom',            LaunchConfiguration('odom_topic')),
                ('rgb/image',       LaunchConfiguration('rgb_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('depth/image',     LaunchConfiguration('depth_topic')),
            ]
        ),
    ])