from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('unitree_ros2') 

    return LaunchDescription([

        # ============================================================
        # 1. Cartographie 
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'lio_slam.launch.py')
            )
        ),

        # ============================================================
        # 2. Évitement d'obstacles 
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='unitree_ros2',
                    executable='obstacle_avoidance.py', 
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
                    executable='mission_manager.py', 
                    output='screen',
                )
            ]
        ),
    ])