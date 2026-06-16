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

    # CONSTANTES - Tuning du robot
    OBSTACLE_DISTANCE = 1.2
    CRITICAL_DISTANCE = 0.7
    MIN_LIDAR_DISTANCE = 0.5
    LIDAR_MAX_X = -0.35
    LIDAR_MAX_Y = 0.30
    LIDAR_BLIND_ZONE_X = 0.5
    LIDAR_BLIND_ZONE_Y = 0.45
    LIDAR_MIN_Z = 0.0
    LIDAR_MAX_Z = 0.25
    LIDAR_CENTER_MARGIN = 0.08
    STARTUP_DELAY = 10.0
    HISTORY_SIZE = 5
    DANGER_999_THRESHOLD = 5
    ESCAPE_PHASE1_DURATION = 4.0
    ESCAPE_PHASE2_DURATION = 2.0
    ESCAPE_TOTAL_TIMEOUT = 15.0
    SPEED_FORWARD = 0.3
    SPEED_MIN_FORWARD = 0.15
    SPEED_MAX_FORWARD = 0.3
    SPEED_BACKWARD = -0.25
    SPEED_TURN_ESCAPE = 0.8
    DIRECTION_LOCK_DURATION = 1.5
    DIRECTION_MARGIN = 0.15
    SMOOTH_BLEND_FACTOR = 0.2

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ===== ETAT =====
        self.obstacle_distance  = self.OBSTACLE_DISTANCE
        self.critical_distance  = self.CRITICAL_DISTANCE
        self.left_dist         = 999.0
        self.right_dist        = 999.0
        self.front_dist        = 999.0

        # Machine d'état
        self.state = 'INIT'
        self.start_time = time.time()
        self.escape_start_time = 0.0
        
        # Navigation
        self.turn_direction = 0.0
        self.left_history = []
        self.right_history = []
        self.direction_lock = 0
        self.lock_time = 0
        
        # Dangers
        self.danger_999_count = 0
        self.last_init_log = 0.0

        # ===== ROS =====
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.pub = self.create_publisher(
            Request, '/api/sport/request', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Started')

    def smooth(self, history):
        """Calcule la moyenne d'une liste"""
        return 999.0 if len(history) == 0 else sum(history) / len(history)
    
    def _create_request_msg(self):
        """Crée un message Request pré-configuré"""
        msg = Request()
        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008
        return msg
    
    def _publish_velocity(self, vx, vz):
        """Publie une commande de velocité"""
        msg = self._create_request_msg()
        velocity = {"x": vx, "y": 0.0, "z": vz}
        msg.parameter = json.dumps(velocity)
        self.pub.publish(msg)
    
    def _get_rotation_direction(self):
        """Retourne direction de rotation (1.0 ou -1.0)"""
        return 1.0 if self.left_dist >= self.right_dist else -1.0

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

            # Filtres de calibration
            if x > self.LIDAR_MAX_X or abs(y) > self.LIDAR_MAX_Y:
                continue
            if abs(x) < self.LIDAR_BLIND_ZONE_X and abs(y) < self.LIDAR_BLIND_ZONE_Y:
                continue
            if z < self.LIDAR_MIN_Z or z > self.LIDAR_MAX_Z:
                continue

            d = math.sqrt(x*x + y*y + z*z)
            if d < self.MIN_LIDAR_DISTANCE:
                continue

            # Zone centrale → voie avant
            if abs(y) < self.LIDAR_CENTER_MARGIN:
                front_dist = min(front_dist, d)
            
            # Zone gauche et droite
            if y >= 0:
                left_dist = min(left_dist, d)
            else:
                right_dist = min(right_dist, d)

        self.front_dist = front_dist

        # Détection danger 999.00 persistant uniquement si la situation est vraiment bloquante
        if front_dist >= 999.0:
            blocked_left = left_dist <= self.OBSTACLE_DISTANCE
            blocked_right = right_dist <= self.OBSTACLE_DISTANCE
            critical_left = left_dist <= self.CRITICAL_DISTANCE
            critical_right = right_dist <= self.CRITICAL_DISTANCE

            if ((critical_left and blocked_right) or
                (critical_right and blocked_left) or
                (blocked_left and blocked_right) or
                (critical_left and critical_right)):
                self.danger_999_count += 1
            else:
                self.danger_999_count = 0
        else:
            self.danger_999_count = 0

        # Lissage avec history
        self.left_history.append(left_dist)
        self.right_history.append(right_dist)

        if len(self.left_history) > self.HISTORY_SIZE:
            self.left_history.pop(0)
        if len(self.right_history) > self.HISTORY_SIZE:
            self.right_history.pop(0)

        self.left_dist = self.smooth(self.left_history)
        self.right_dist = self.smooth(self.right_history)

        # Log une fois par seconde
        if time.time() - self.last_init_log > 1.0:
            self.get_logger().info(
                f'front={front_dist:.2f}m | gauche={self.left_dist:.2f}m | droite={self.right_dist:.2f}m'
            )
            self.last_init_log = time.time()

    # ===================================
    # CONTROL LOOP — décision principale
    # ===================================
    def control_loop(self):

        elapsed = time.time() - self.start_time

        # Phase initialisation
        if self.state == 'INIT':
            if elapsed < self.STARTUP_DELAY:
                if self.front_dist < self.CRITICAL_DISTANCE:
                    self.get_logger().info('🚨 Obstacle trop proche au démarrage')
                    self.state = 'ESCAPE_STARTUP'
                    self.escape_start_time = time.time()
                elif time.time() - self.last_init_log > 1.0:
                    self.get_logger().info(f'Initialisation capteurs... {elapsed:.1f}s')
                    self.last_init_log = time.time()
                return
            else:
                self.state = 'RUNNING'
                self.get_logger().info('✅ Navigation started')

        # Mode escape startup
        if self.state == 'ESCAPE_STARTUP':
            self._escape_sequence(self.escape_start_time, 'startup')
            return

        # Mode escape danger pendant navigation
        if self.state == 'ESCAPE_DANGER':
            self._escape_sequence(self.escape_start_time, 'danger')
            return

        # DANGER CRITIQUE: 999.00 persistant
        if self.danger_999_count >= self.DANGER_999_THRESHOLD:
            self.get_logger().info(
                f'🚨🚨 DANGER CRITIQUE ({self.danger_999_count} obs 999) - RECUL URGENT'
            )
            self._publish_velocity(0.0, 0.0)
            self.state = 'ESCAPE_DANGER'
            self.escape_start_time = time.time()
            return

        # Mode normal: CAS 1/2/3
        self._navigate_normal()

    def _navigate_normal(self):
        """Navigation en mode normal (3 cas)"""
        front = self.front_dist
        left = self.left_dist
        right = self.right_dist

        # CAS 1: voie libre
        if front > self.OBSTACLE_DISTANCE:
            self._publish_velocity(self.SPEED_FORWARD, 0.0)
            self.turn_direction = 0.0
            return

        # si le front est invalide mais une des voies latérales est libre, on contourne
        if front >= 999.0 and (left > self.OBSTACLE_DISTANCE or right > self.OBSTACLE_DISTANCE):
            self.get_logger().info('🟡 Front invalide, contournement latéral disponible')
            vz = self._get_rotation_direction() * self.SPEED_TURN_ESCAPE * 0.5
            self._publish_velocity(self.SPEED_MIN_FORWARD, vz)
            return

        # CAS 2: obstacle détecté à distance
        if front > self.CRITICAL_DISTANCE:
            self._navigate_with_obstacle(front, left, right)
            return

        # CAS 3: danger immédiat
        self.get_logger().info(f'🚨 DANGER {front:.2f}m - RECUL')
        self._publish_velocity(0.0, 0.0)
        self.state = 'ESCAPE_DANGER'
        self.escape_start_time = time.time()

    def _navigate_with_obstacle(self, front, left, right):
        """Gère la navigation avec obstacle détecté (CAS 2)"""
        # Gestion du verrou de direction
        if self.direction_lock != 0:
            if time.time() - self.lock_time < self.DIRECTION_LOCK_DURATION:
                self.turn_direction = float(self.direction_lock)
            else:
                self.direction_lock = 0
        else:
            self._update_turn_direction(left, right)

        # Calcul des vitesses proportionnelles
        ratio = (front - self.CRITICAL_DISTANCE) / (self.OBSTACLE_DISTANCE - self.CRITICAL_DISTANCE)
        vx = self.SPEED_MIN_FORWARD + ratio * (self.SPEED_MAX_FORWARD - self.SPEED_MIN_FORWARD)
        vz = self.turn_direction * (0.5 + 0.5 * ratio)

        self._publish_velocity(vx, vz)

    def _update_turn_direction(self, left, right):
        """Met à jour la direction de rotation en fonction des obstacles"""
        if self.turn_direction > 0:
            if left < self.CRITICAL_DISTANCE:
                self.turn_direction = -1.0
                self.direction_lock = -1
                self.lock_time = time.time()
        elif self.turn_direction < 0:
            if right < self.CRITICAL_DISTANCE:
                self.turn_direction = 1.0
                self.direction_lock = 1
                self.lock_time = time.time()
        else:
            if left > right + self.DIRECTION_MARGIN:
                new_direction = 1.0
                self.direction_lock = 1
            elif right > left + self.DIRECTION_MARGIN:
                new_direction = -1.0
                self.direction_lock = -1
            else:
                new_direction = 1.0
                self.direction_lock = 1

            self.lock_time = time.time()
            self.turn_direction = (self.turn_direction * (1 - self.SMOOTH_BLEND_FACTOR) +
                                   new_direction * self.SMOOTH_BLEND_FACTOR)

    def _escape_sequence(self, start_time, escape_type):
        """Séquence unifiée de dégagement: recul + rotation + vérification
        
        Args:
            start_time: timestamp du début du dégagement
            escape_type: 'startup' ou 'danger' pour les logs
        """
        elapsed = time.time() - start_time
        phase_times = {
            'startup': (2.0, 4.0),  # phase1_end, phase2_end
            'danger': (self.ESCAPE_PHASE1_DURATION, 
                      self.ESCAPE_PHASE1_DURATION + self.ESCAPE_PHASE2_DURATION)
        }
        phase1_end, phase2_end = phase_times.get(escape_type, phase_times['danger'])

        # Phase 1: Recul
        if elapsed < phase1_end:
            self.get_logger().info(f'🚨 Recul dégagement ({elapsed:.1f}s)')
            self._publish_velocity(self.SPEED_BACKWARD, 0.0)

        # Phase 2: Rotation
        elif elapsed < phase2_end:
            self.get_logger().info('↩️ Rotation dégagement')
            vz = self._get_rotation_direction() * self.SPEED_TURN_ESCAPE
            self._publish_velocity(0.0, vz)

        # Phase 3: Vérification + retry
        else:
            if elapsed > self.ESCAPE_TOTAL_TIMEOUT:
                self.get_logger().warn('⏱️ Timeout dégagement - forcer continuation')
                self.state = 'RUNNING'
                return

            # Vérifier voie libre
            if (self.front_dist > self.OBSTACLE_DISTANCE and
                self.left_dist > self.CRITICAL_DISTANCE and
                self.right_dist > self.CRITICAL_DISTANCE):
                self.get_logger().info('✅ Dégagement réussi!')
                self.state = 'RUNNING'
            else:
                # Continuer rotation
                self.get_logger().info(
                    f'Voie obstruée (F:{self.front_dist:.2f} L:{self.left_dist:.2f} R:{self.right_dist:.2f}) - rotation'
                )
                vz = self._get_rotation_direction() * self.SPEED_TURN_ESCAPE
                self._publish_velocity(0.0, vz)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()