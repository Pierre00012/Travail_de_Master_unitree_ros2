#!/usr/bin/env python3
"""
calibrate_zones.py
Capture la signature LiDAR de référence pour chaque zone autour du robot.
Le robot doit être placé en zone dégagée, immobile.
Les résultats sont sauvegardés dans reference_zones.json
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math
import json
import time
import os

ZONES = ['front', 'front_left', 'front_right', 'left', 'right', 'rear']
GRID_RESOLUTION = 0.10
CAPTURE_DURATION = 10.0  # secondes de capture par zone
OUTPUT_FILE = os.path.expanduser(
    '~/unitree_ros2/unitree_ros2/src/unitree_ros2/scripts/reference_zones.json')

class ZoneCalibrator(Node):

    def __init__(self):
        super().__init__('zone_calibrator')

        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.capturing      = False
        self.capture_start  = 0.0
        self.zone_maps      = {z: {} for z in ZONES}
        self.frame_count    = 0
        self.current_frames = 0

        # Démarre la calibration après 3s
        self.timer = self.create_timer(3.0, self.start_calibration)
        self.get_logger().info(
            '📐 Calibrateur de zones démarré')
        self.get_logger().info(
            '⚠️  Placez le robot en zone DÉGAGÉE et attendez 3s...')

    def point_to_cell(self, x, y, z):
        cx = int(round(x / GRID_RESOLUTION))
        cy = int(round(y / GRID_RESOLUTION))
        cz = int(round(z / GRID_RESOLUTION))
        return (cx, cy, cz)

    def classify_zone(self, x, y):
        if x > 0.0:
            return 'rear'
        if x <= -0.35:
            if y > 0.10:
                return 'front_left'
            elif y < -0.10:
                return 'front_right'
            else:
                return 'front'
        if y > 0:
            return 'left'
        return 'right'

    def start_calibration(self):
        self.timer.cancel()
        self.capturing     = True
        self.capture_start = time.time()
        self.get_logger().info(
            f'📸 Capture référence pendant {CAPTURE_DURATION:.0f}s...')

    def lidar_callback(self, msg):
        if not self.capturing:
            return

        elapsed = time.time() - self.capture_start

        if elapsed > CAPTURE_DURATION:
            self.save_reference()
            return

        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

        self.current_frames += 1
        points_per_zone = {z: 0 for z in ZONES}

        for p in points:
            x, y, z = p

            # Exclut le corps du robot
            if abs(x) < 0.5 and abs(y) < 0.45:
                continue
            if z < 0.0 or z > 0.60:
                continue
            d = math.sqrt(x*x + y*y + z*z)
            if d < 0.5 or d > 4.0:
                continue

            zone = self.classify_zone(x, y)
            cell = self.point_to_cell(x, y, z)

            # Accumule les occurrences par cellule
            if cell not in self.zone_maps[zone]:
                self.zone_maps[zone][cell] = 0
            self.zone_maps[zone][cell] += 1
            points_per_zone[zone] += 1

        # Log progression toutes les secondes
        if self.current_frames % 15 == 0:
            self.get_logger().info(
                f't={elapsed:.1f}s | frames={self.current_frames} | '
                f'front={points_per_zone["front"]} pts | '
                f'fl={points_per_zone["front_left"]} pts | '
                f'fr={points_per_zone["front_right"]} pts | '
                f'G={points_per_zone["left"]} pts | '
                f'D={points_per_zone["right"]} pts'
            )

    def save_reference(self):
        self.capturing = False

        # Convertit les tuples en strings pour JSON
        output = {}
        stats  = {}

        for zone, cells in self.zone_maps.items():
            # Garde seulement les cellules vues au moins 3 fois
            # (filtre le bruit — une cellule vue 1 seule fois = artefact)
            stable_cells = {
                f"{cx},{cy},{cz}": count
                for (cx, cy, cz), count in cells.items()
                if count >= 3
            }
            output[zone] = stable_cells
            stats[zone]  = len(stable_cells)

        # Sauvegarde
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(output, f, indent=2)

        self.get_logger().info('=' * 50)
        self.get_logger().info(f'✅ Référence sauvegardée : {OUTPUT_FILE}')
        self.get_logger().info(f'   Frames capturées : {self.current_frames}')
        for zone, count in stats.items():
            self.get_logger().info(f'   {zone:12s} : {count:4d} cellules stables')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            'Vous pouvez maintenant lancer obstacle_avoidance_v3.py')

        rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(ZoneCalibrator())

if __name__ == '__main__':
    main()