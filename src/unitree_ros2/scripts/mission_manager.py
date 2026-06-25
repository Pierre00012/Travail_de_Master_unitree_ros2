#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from nav_msgs.msg import Odometry
import json
import time
import math

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ==================== CONFIG ====================
        self.MISSION_DURATION = 30.0          # secondes
        self.RETURN_LINEAR_SPEED = 0.22
        self.RETURN_ANGULAR_SPEED = 1.2       # Rotation plus forte
        self.POSITION_TOLERANCE = 0.25
        self.ANGLE_TOLERANCE = 0.20

        # Remplacement par time.monotonic() pour la stabilité réseau
        self.start_time = time.monotonic()
        self.return_phase = False
        self.rotation_180_phase = False       # <-- NOUVEAU : Phase intermédiaire de demi-tour
        self.mission_completed = False

        self.trajectory = []
        self.current_pose = None
        self.return_target_index = 0
        self.yaw_target_180 = 0.0             # <-- NOUVEAU : Cible de la rotation de départ

        # Changement du nom du callback pour plus de clarté
        self.last_sample = time.monotonic()

        self.cmd_pub = self.create_publisher(Request, '/api/sport/request', 10)

        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_callback, 10)

        self.create_timer(0.5, self.mission_monitoring_loop)
        self.create_timer(0.1, self.return_control_loop)

        self.get_logger().info('MISSION MANAGER DÉMARRÉ - Mode Fil d\'Ariane actif')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

        self.current_pose = (x, y, yaw)

        # On n'enregistre la trajectoire que pendant l'aller (Niveau 1)
        if self.return_phase or self.rotation_180_phase:
            return

        now = time.monotonic()
        if now - self.last_sample > 0.4:
            self.trajectory.append((x, y, yaw))
            self.last_sample = now

    def mission_monitoring_loop(self):
        if self.return_phase or self.rotation_180_phase or self.mission_completed:
            return

        elapsed = time.monotonic() - self.start_time

        if int(elapsed) % 3 == 0:
            self.get_logger().info(f'[MISSION] Temps écoulé : {elapsed:.1f}s / {self.MISSION_DURATION}s')

        if elapsed >= self.MISSION_DURATION:
            self.get_logger().warn('='*70)
            self.get_logger().warn('⏰ FIN DE MISSION → SÉQUENCE DEMI-TOUR 180°')
            self.get_logger().warn('='*70)
            self.start_rotation_180_phase()

    def start_rotation_180_phase(self):
        """Déclenche le demi-tour pur sur place avant de suivre le chemin"""
        # ============================================================
        # ⚡ TUER LE SCRIPT D'ÉVITEMENT D'OBSTACLE POUR ÉVITER LE CONFLIT
        # ============================================================
        import os
        # Cette commande Linux cherche le script d'évitement et le coupe proprement
        os.system("pkill -f obstacle_avoidance.py")
        self.get_logger().warn('🛑 SYSTÈME : Évitement d\'obstacle désactivé pour le retour.')
        # ============================================================
        if len(self.trajectory) < 5:
            self.get_logger().error("Trajectoire trop courte !")
            self._emergency_stop()
            self.mission_completed = True
            return

        if self.current_pose is None:
            self.get_logger().error("Pas d'odométrie reçue, impossible de pivoter !")
            self._emergency_stop()
            self.mission_completed = True
            return

        # Calcul du cap inverse à l'orientation actuelle
        _, _, cyaw = self.current_pose
        self.yaw_target_180 = math.atan2(math.sin(cyaw + math.pi), math.cos(cyaw + math.pi))
        
        self.rotation_180_phase = True
        self.get_logger().info('🔄 DEMI-TOUR DIRECT : Pivotement sur place vers le chemin du retour...')

    def return_control_loop(self):
        if self.mission_completed or self.current_pose is None:
            return

        # 1. GESTION DU DEMI-TOUR À 180° DE DÉPART
        if self.rotation_180_phase:
            _, _, cyaw = self.current_pose
            angle_error = math.atan2(math.sin(self.yaw_target_180 - cyaw), math.cos(self.yaw_target_180 - cyaw))

            if abs(angle_error) < self.ANGLE_TOLERANCE:
                # 180° Réussi ! On bascule sur le suivi de trajectoire inversé
                self.rotation_180_phase = False
                self.return_phase = True
                self.trajectory.reverse() # Inversion du fil d'Ariane
                self.return_target_index = 0
                self.get_logger().warn('✅ DEMI-TOUR RÉUSSI : Démarrage du suivi du fil d\'Ariane.')
                return

            # Envoi de la vitesse de rotation pure sur place
            vz = self.RETURN_ANGULAR_SPEED if angle_error > 0 else -self.RETURN_ANGULAR_SPEED
            self._publish_velocity(0.0, vz)
            return

        # 2. SUIVI DU FIL D'ARIANE EN SENS INVERSE (SUIVANT TON CODE)
        if not self.return_phase:
            return

        if self.return_target_index >= len(self.trajectory):
            self.get_logger().warn('🎯 RETOUR TERMINÉ AVEC SUCCÈS AU REPAIRE D\'ORIGINE')
            self._emergency_stop()
            self.mission_completed = True
            return

        tx, ty, _ = self.trajectory[self.return_target_index]
        cx, cy, cyaw = self.current_pose

        dx = tx - cx
        dy = ty - cy
        distance = math.hypot(dx, dy)

        if distance < self.POSITION_TOLERANCE:
            self.get_logger().info(f'📍 Point {self.return_target_index}/{len(self.trajectory)} atteint')
            self.return_target_index += 1
            return

        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_angle - cyaw), math.cos(target_angle - cyaw))

        if abs(angle_error) > self.ANGLE_TOLERANCE:
            vz = self.RETURN_ANGULAR_SPEED if angle_error > 0 else -self.RETURN_ANGULAR_SPEED
            self._publish_velocity(0.0, vz)
            self.get_logger().info(f'↩️ Alignement vers point suivant | err={math.degrees(angle_error):.1f}°')
        else:
            vz = max(-0.6, min(0.6, angle_error * 2.5))
            self._publish_velocity(self.RETURN_LINEAR_SPEED, vz)
            self.get_logger().info(f'➡️ Avance | dist={distance:.2f}m')

    def _publish_velocity(self, vx, vz):
        """Centralisation de l'envoi vers le SDK Sport (Corrigé à 1008)"""
        req = Request()
        req.header.identity.id = 1
        req.header.identity.api_id = 1008  # !!! CORRIGÉ : 1008 = Vitesse Sport (1005 faisait coucher le robot)
        vel = {"x": vx, "y": 0.0, "z": vz}
        req.parameter = json.dumps(vel)
        self.cmd_pub.publish(req)

    def _emergency_stop(self):
        """Force l'immobilisation complète du robot (Corrigé à 1008)"""
        req = Request()
        req.header.identity.id = 1
        req.header.identity.api_id = 1008  # !!! CORRIGÉ : 1008
        req.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self.cmd_pub.publish(req)

    def destroy_node(self):
        self._emergency_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission arrêtée manuellement')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()