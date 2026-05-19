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

        # pour gerer le demarrage 
        self.start_time = time.time()
        self.startup_delay = 10.0  # secondes

        self.turn_direction    = 0.0    # 0 = tout droit
        self.startup_escape = False

        # pour gerer les virages 
        self.history_size = 5
        self.left_history = []
        self.right_history = []

        self.direction_lock = 0      # 0 = libre, 1 = gauche, -1 = droite
        self.lock_time = 0
        self.lock_duration = 1.5     # secondes

        # temps début manoeuvre
        self.escape_start_time = 0.0

        # ===== ROS =====
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.pub = self.create_publisher(
            Request, '/api/sport/request', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Started')

    # ==========================
    # moyenne history
    # ==========================
    def smooth(self, history):
        if len(history) == 0:
            return 999.0
        return sum(history) / len(history)

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

        # ===== HISTORY (lissage) =====
        self.left_history.append(left_dist)
        self.right_history.append(right_dist)

        if len(self.left_history) > self.history_size:
            self.left_history.pop(0)

        if len(self.right_history) > self.history_size:
            self.right_history.pop(0)

        self.left_dist = self.smooth(self.left_history)
        self.right_dist = self.smooth(self.right_history)

        self.closest_distance = min(front_dist, self.left_dist, self.right_dist)

        self.get_logger().info(
            f'front={front_dist:.2f}m | gauche={self.left_dist:.2f}m | droite={self.right_dist:.2f}m'
        )

    # ===================================
    # CONTROL LOOP — toujours en mouvement
    # ===================================
    def control_loop(self):

        elapsed = time.time() - self.start_time

        # ==========================
        # MODE DEGAGEMENT
        # ==========================
        if self.startup_escape:
            self.danger_startup()
            return

        # ==========================
        # DELAI INITIALISATION
        # ==========================
        if elapsed < self.startup_delay:

            if self.front_dist < self.critical_distance:

                self.get_logger().info(
                    '🚨 Obstacle trop proche au démarrage'
                )

                self.startup_escape = True
                self.escape_start_time = time.time()
                return

            self.get_logger().info(
                f'Initialisation capteurs... {elapsed:.1f}/10s'
            )

            return

        msg = Request()
        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008

        front = self.front_dist
        left  = self.left_dist
        right = self.right_dist

        # ==========================
        # CAS 1 : voie totalement libre
        # ==========================
        if front > self.obstacle_distance:

            vx = 0.3
            vz = 0.0
            self.turn_direction = 0.0

        # ==========================
        # CAS 2 : obstacle détecté à distance
        # ==========================
        elif front > self.critical_distance:

            margin = 0.15

            # ==========================
            # VERROU DE DIRECTION
            # ==========================
            if self.direction_lock != 0:
                if time.time() - self.lock_time < self.lock_duration:
                    self.turn_direction = float(self.direction_lock)
                else:
                    self.direction_lock = 0

            else:

                # maintien direction
                if self.turn_direction > 0:
                    if self.left_dist < self.critical_distance:
                        self.turn_direction = -1.0
                        self.direction_lock = -1
                        self.lock_time = time.time()

                elif self.turn_direction < 0:
                    if self.right_dist < self.critical_distance:
                        self.turn_direction = 1.0
                        self.direction_lock = 1
                        self.lock_time = time.time()

                else:

                    if left > right + margin:
                        new_direction = 1.0
                        self.direction_lock = 1
                        self.lock_time = time.time()

                    elif right > left + margin:
                        new_direction = -1.0
                        self.direction_lock = -1
                        self.lock_time = time.time()

                    else:
                        new_direction = 1.0
                        self.direction_lock = 1
                        self.lock_time = time.time()

                    self.turn_direction = (
                        self.turn_direction * 0.8 +
                        new_direction * 0.2
                    )

            ratio = (front - self.critical_distance) / \
                    (self.obstacle_distance - self.critical_distance)

            vx = 0.15 + ratio * 0.15
            vz = self.turn_direction * 0.7

        # ==========================
        # CAS 3 : danger immédiat
        # ==========================
        else:

            if self.left_dist >= self.right_dist:
                self.turn_direction = 1.0
            else:
                self.turn_direction = -1.0

            self.get_logger().info(
                f'🚨 DANGER {front:.2f}m — rotation sur place'
            )

            vx = 0.0
            vz = self.turn_direction * 0.8

        velocity = {"x": vx, "y": 0.0, "z": vz}
        msg.parameter = json.dumps(velocity)
        self.pub.publish(msg)

    # ==========================
    # danger au démarrage
    # ==========================
    def danger_startup(self):

        msg = Request()
        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008

        elapsed_escape = time.time() - self.escape_start_time

        if elapsed_escape < 2.0:

            self.get_logger().info(
                '🚨 Danger immédiat au démarrage -> recul'
            )

            velocity = {"x": -0.25, "y": 0.0, "z": 0.0}

        elif elapsed_escape < 4.0:

            self.get_logger().info(
                '↩️ Rotation de dégagement'
            )

            if self.left_dist >= self.right_dist:
                vz = 0.8
            else:
                vz = -0.8

            velocity = {"x": 0.0, "y": 0.0, "z": vz}

        else:

            self.get_logger().info(
                '✅ Dégagement terminé'
            )

            self.startup_escape = False
            return

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