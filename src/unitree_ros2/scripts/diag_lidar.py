#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math

class DiagLidar(Node):
    def __init__(self):
        super().__init__('diag_lidar')
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cb, 10)

    def cb(self, msg):
        points = list(point_cloud2.read_points(
            msg, field_names=("x","y","z"), skip_nans=True))

        # Tous les points devant (x négatif) sans aucun filtre
        front = []
        for p in points:
            x, y, z = p
            if x > -0.1:
                continue
            if abs(y) > 0.30:
                continue
            if z < 0.0 or z > 0.25:
                continue
            d = math.sqrt(x*x + y*y + z*z)
            front.append((d, x, y, z))

        front.sort()  # tri par distance croissante

        if front:
            self.get_logger().info('--- 5 points les plus proches ---')
            for d, x, y, z in front[:5]:
                self.get_logger().info(
                    f'  d={d:.3f}m | x={x:.3f} | y={y:.3f} | z={z:.3f}')
        else:
            self.get_logger().info('Aucun point devant')

def main():
    rclpy.init()
    rclpy.spin(DiagLidar())

if __name__ == '__main__':
    main()
