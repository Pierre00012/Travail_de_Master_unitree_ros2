#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math

class DiagZ(Node):
    def __init__(self):
        super().__init__('diag_z')
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cb, 10)

    def cb(self, msg):
        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

        front = []
        for p in points:
            x, y, z = p

            # Filtres de base uniquement
            if x > -0.35:
                continue
            if abs(y) > 0.30:
                continue
            if abs(x) < 0.8 and abs(y) < 0.45:
                continue

            # PAS de filtre z → on veut tout voir
            d = math.sqrt(x*x + y*y + z*z)
            if d < 0.5 or d > 2.0:
                continue

            front.append((d, x, y, z))

        front.sort()

        if front:
            self.get_logger().info('--- Points proches par hauteur ---')
            for d, x, y, z in front[:8]:
                self.get_logger().info(
                    f'  d={d:.2f}m | x={x:.2f} | y={y:.2f} | z={z:.2f}')
        else:
            self.get_logger().info('Aucun point devant')

def main():
    rclpy.init()
    rclpy.spin(DiagZ())

if __name__ == '__main__':
    main()