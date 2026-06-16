from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Récupération du dossier partagé de ton package
    pkg_dir = get_package_share_directory('unitree_ros2') 

    return LaunchDescription([

        # ============================================================
        # 1. Cartographie (Ton fichier LIO-SLAM existant)
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'lio_slam.launch.py')
            )
        ),

        # ============================================================
        # 2. Évitement d'obstacles (Scripts Python installés)
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='unitree_ros2',
                    executable='obstacle_avoidance.py', # Doit correspondre au nom dans CMakeLists
                    name='obstacle_avoidance',
                    output='screen',
                )
            ]
        ),

        # ============================================================
        # 3. Le Manager (Superviseur)
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='unitree_ros2',
                    executable='mission_manager.py', # Doit correspondre au nom dans CMakeLists
                    name='mission_manager',
                    output='screen',
                )
            ]
        ),
    ])