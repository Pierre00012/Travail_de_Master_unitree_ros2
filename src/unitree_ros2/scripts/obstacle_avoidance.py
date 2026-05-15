#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from unitree_api.msg import Request
import json
import math
import time
class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ===== PARAMETRES =====
        self.obstacle_distance  = 1.2   # détection anticipée
        self.critical_distance  = 0.7   # danger immédiat

        # ===== ETAT =====
        self.closest_distance  = 999.0
        self.left_dist         = 999.0
        self.right_dist        = 999.0
        self.front_dist        = 999.0
        self.start_time = time.time()
        self.startup_delay = 10.0  # secondes
        self.turn_direction    = 0.0    # 0 = tout droit

        # ===== ROS =====
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.pub = self.create_publisher(
            Request, '/api/sport/request', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Started')

    # ===================================
    # LIDAR — analyse 3 zones : gauche / centre / droite
    # ===================================
    def lidar_callback(self, msg):

        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

        front_dist = 999.0
        left_dist  = 999.0
        right_dist = 999.0

        for p in points:
            x, y, z = p

            # Tes filtres de calibration
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

            # Zone centrale ±8cm → voie avant
            if abs(y) < 0.08:
                if d < front_dist:
                    front_dist = d

            # Zone gauche (y > 0)
            if y >= 0:
                if d < left_dist:
                    left_dist = d

            # Zone droite (y < 0)
            if y < 0:
                if d < right_dist:
                    right_dist = d

        self.front_dist = front_dist
        self.left_dist  = left_dist
        self.right_dist = right_dist

        # Distance minimale globale
        self.closest_distance = min(front_dist, left_dist, right_dist)

        self.get_logger().info(
            f'front={front_dist:.2f}m | '
            f'gauche={left_dist:.2f}m | '
            f'droite={right_dist:.2f}m'
        )

    # ===================================
    # CONTROL LOOP — toujours en mouvement
    # ===================================
    def control_loop(self):

        # attendre 10 secondes avant mouvement
        elapsed = time.time() - self.start_time
        if elapsed < self.startup_delay:
            self.get_logger().info(
                f'Initialisation capteurs... {elapsed:.1f}/10s'
            )
            msg = Request()
            msg.header.identity.id = 1
            msg.header.identity.api_id = 1008

            # robot immobile
            velocity = {"x": 0.0, "y": 0.0, "z": 0.0}

            msg.parameter = json.dumps(velocity)
            self.pub.publish(msg)

            return

        msg = Request()
        msg.header.identity.id     = 1
        msg.header.identity.api_id = 1008

        front = self.front_dist
        left  = self.left_dist
        right = self.right_dist

        # ==========================
        # CAS 1 : voie totalement libre
        # ==========================
        if front > self.obstacle_distance:
            # Avance tout droit, vitesse normale
            vx = 0.3
            vz = 0.0
            self.turn_direction = 0.0

        # ==========================
        # CAS 2 : obstacle détecté à distance
        # Amorce un virage SANS s'arrêter
        # ==========================
        elif front > self.critical_distance:

            # Choisit le côté le plus libre
            # Si en train de tourner, vérifie que c'est toujours ok
            if self.turn_direction > 0:
                # Tournait à gauche — gauche bloquée ? → droite
                if left < self.critical_distance:
                    self.turn_direction = -1.0
                    self.get_logger().info('↩️  Gauche bloquée → bascule à droite')
            elif self.turn_direction < 0:
                # Tournait à droite — droite bloquée ? → gauche
                if right < self.critical_distance:
                    self.turn_direction = 1.0
                    self.get_logger().info('↩️  Droite bloquée → bascule à gauche')
            else:
                # Nouveau choix : côté le plus dégagé
                if left >= right:
                    self.turn_direction = 1.0
                    self.get_logger().info(
                        f'🔄 Virage GAUCHE anticipé '
                        f'(gauche={left:.2f}m > droite={right:.2f}m)')
                else:
                    self.turn_direction = -1.0
                    self.get_logger().info(
                        f'🔄 Virage DROITE anticipé '
                        f'(droite={right:.2f}m > gauche={left:.2f}m)')

            # Avance EN tournant — vitesse proportionnelle à la distance
            ratio = (front - self.critical_distance) / \
                    (self.obstacle_distance - self.critical_distance)
            vx = 0.15 + ratio * 0.15   # entre 0.15 et 0.30 m/s
            vz = self.turn_direction * 0.7

        # ==========================
        # CAS 3 : danger immédiat
        # Tourne sur place sans avancer
        # ==========================
        else:
            if left >= right:
                self.turn_direction = 1.0
            else:
                self.turn_direction = -1.0

            self.get_logger().info(
                f'🚨 DANGER {front:.2f}m — rotation sur place')

            vx = 0.0
            vz = self.turn_direction * 0.8

        velocity = {"x": vx, "y": 0.0, "z": vz}
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