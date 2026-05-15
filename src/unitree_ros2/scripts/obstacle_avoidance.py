#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from unitree_api.msg import Request
import json
import math

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.obstacle_distance = 1.2

        self.turning        = False
        self.turn_direction = 1.0

        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.pub = self.create_publisher(
            Request, '/api/sport/request', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.obstacle_detected  = False
        self.closest_distance   = 999.0
        self.left_clear_dist    = 999.0   # distance libre à gauche
        self.right_clear_dist   = 999.0   # distance libre à droite

        self.get_logger().info('Obstacle Avoidance Started')

    def lidar_callback(self, msg):

        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

        min_dist    = 999.0
        left_dist   = 999.0   # y > 0  = gauche
        right_dist  = 999.0   # y < 0  = droite

        for p in points:
            x, y, z = p

            if x > -0.35:
                continue
            if abs(y) > 0.30:
                continue
            if abs(x) < 0.5 and abs(y) < 0.45:
                continue
            if z < 0.0 or z > 0.25:
                continue

            d = math.sqrt(x*x + y*y + z*z)
            if d < 0.5:
                continue

            # Distance minimale globale (voie avant)
            if d < min_dist:
                min_dist = d

            # Séparation gauche / droite sur l'axe y
            if y >= 0 and d < left_dist:
                left_dist = d
            elif y < 0 and d < right_dist:
                right_dist = d

        self.closest_distance  = min_dist
        self.left_clear_dist   = left_dist
        self.right_clear_dist  = right_dist
        self.obstacle_detected = min_dist < self.obstacle_distance

        if self.obstacle_detected:
            direction = 'gauche' if left_dist > right_dist else 'droite'
            self.get_logger().info(
                f'⚠️  OBSTACLE à {min_dist:.2f}m | '
                f'gauche={left_dist:.2f}m droite={right_dist:.2f}m | '
                f'→ tourne {direction}')
        else:
            self.get_logger().info(
                f'✅ Voie libre — distance min: {min_dist:.2f}m')

    def control_loop(self):

        msg = Request()
        msg.header.identity.id     = 1
        msg.header.identity.api_id = 1008

        # ==========================
        # EN TRAIN DE TOURNER
        # Tourne jusqu'à ce que la voie soit libre
        # ==========================
        if self.turning:
            # Voie libre devant → arrêter le virage
            if self.closest_distance > self.obstacle_distance:
                self.turning = False
                self.get_logger().info('✅ Voie libre — reprise en avant')
                velocity = {"x": 0.25, "y": 0.0, "z": 0.0}
            else:
                # Continue de tourner
                velocity = {"x": 0.0, "y": 0.0, "z": self.turn_direction * 0.6}

        # ==========================
        # OBSTACLE → CHOISIR LA MEILLEURE DIRECTION
        # ==========================
        elif self.obstacle_detected:
            self.turning = True

            # Tourne vers le côté le plus dégagé
            if self.left_clear_dist >= self.right_clear_dist:
                self.turn_direction = 1.0   # gauche
                self.get_logger().info(
                    f'🔄 Virage GAUCHE '
                    f'(gauche={self.left_clear_dist:.2f}m > '
                    f'droite={self.right_clear_dist:.2f}m)')
            else:
                self.turn_direction = -1.0  # droite
                self.get_logger().info(
                    f'🔄 Virage DROITE '
                    f'(droite={self.right_clear_dist:.2f}m > '
                    f'gauche={self.left_clear_dist:.2f}m)')

            velocity = {"x": 0.0, "y": 0.0, "z": self.turn_direction * 0.6}

        # ==========================
        # VOIE LIBRE → AVANCER
        # ==========================
        else:
            velocity = {"x": 0.25, "y": 0.0, "z": 0.0}

        msg.parameter = json.dumps(velocity)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()