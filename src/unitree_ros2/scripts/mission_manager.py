#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from nav_msgs.msg import Odometry
import json
import time
import math
import os  

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ==================== CONFIG ====================
        self.MISSION_DURATION = 25.0          # secondes
        self.RETURN_LINEAR_SPEED = 0.25       
        self.RETURN_ANGULAR_SPEED = 1.2       # Vitesse de rotation robuste
        
        # AJUSTEMENT CRITIQUE : Tolérance élargie à 38 cm 
        self.POSITION_TOLERANCE = 0.38        
        self.ANGLE_TOLERANCE = 0.18           

        self.start_time = time.monotonic()
        self.return_phase = False
        self.rotation_180_phase = False       # Phase intermédiaire de demi-tour
        self.mission_completed = False

        self.trajectory = []
        self.current_pose = None
        self.return_target_index = 0
        self.yaw_target_180 = 0.0             

        self.last_sample = time.monotonic()

        self.cmd_pub = self.create_publisher(Request, '/api/sport/request', 10)

        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_callback, 10)

        self.create_timer(0.5, self.mission_monitoring_loop)
        self.create_timer(0.1, self.return_control_loop)

        self.get_logger().info('🚀 MISSION MANAGER DÉMARRÉ - Mode Fil d\'Ariane fluide actif')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

        self.current_pose = (x, y, yaw)

        if self.return_phase or self.rotation_180_phase or self.mission_completed:
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
        if len(self.trajectory) < 5:
            self.get_logger().error("Trajectoire aller trop courte pour générer un retour !")
            self._emergency_stop()
            self.mission_completed = True
            return

        if self.current_pose is None:
            self.get_logger().error("Pas d'odométrie reçue, impossible de pivoter !")
            self._emergency_stop()
            self.mission_completed = True
            return

        # Libération immédiate du topic /api/sport/request
        os.system("pkill -f obstacle_avoidance.py")
        self.get_logger().warn('🛑 SYSTÈME : Évitement d\'obstacle coupé pour libérer la bande passante.')

        _, _, cyaw = self.current_pose
        self.yaw_target_180 = math.atan2(math.sin(cyaw + math.pi), math.cos(cyaw + math.pi))
        
        self.rotation_180_phase = True
        self.get_logger().info('🔄 DEMI-TOUR DIRECT : Pivotement sur place vers le chemin du retour...')

    def return_control_loop(self):
        if self.mission_completed or self.current_pose is None:
            return

        # 1. PHASE DE DEMI-TOUR INITIAL À 180°
        if self.rotation_180_phase:
            _, _, cyaw = self.current_pose
            angle_error = math.atan2(math.sin(self.yaw_target_180 - cyaw), math.cos(self.yaw_target_180 - cyaw))

            if abs(angle_error) < self.ANGLE_TOLERANCE:
                self.rotation_180_phase = False
                self.return_phase = True
                self.trajectory.reverse()  # Inversion du fil d'Ariane
                self.return_target_index = 0
                self.get_logger().warn('✅ DEMI-TOUR RÉUSSI : Démarrage direct du suivi de ligne.')
            else:
                vz = self.RETURN_ANGULAR_SPEED if angle_error > 0 else -self.RETURN_ANGULAR_SPEED
                self._publish_velocity(0.0, vz)
                return

        # 2. RETOUR LE LONG DU FIL D'ARIANE (REJOUR LES POINTS ENREGISTRÉS EN SENS INVERSE)
        if not self.return_phase:
            return

        if self.return_target_index >= len(self.trajectory):
            self.get_logger().warn('🎯 RETOUR TERMINÉ AVEC SUCCÈS AU POINT DE DÉPART')
            self._emergency_stop()
            self.mission_completed = True
            return

        tx, ty, _ = self.trajectory[self.return_target_index]
        cx, cy, cyaw = self.current_pose

        dx = tx - cx
        dy = ty - cy
        distance = math.hypot(dx, dy)

        # Nettoyage instantané des points morts sous la nouvelle tolérance (0.38m)
        while distance < self.POSITION_TOLERANCE:
            self.get_logger().info(f'📍 Point {self.return_target_index} validé ({distance:.2f}m). Passage au suivant.')
            self.return_target_index += 1
            
            if self.return_target_index >= len(self.trajectory):
                self.get_logger().warn('🎯 RETOUR TERMINÉ : Fin de la liste de points.')
                self._emergency_stop()
                self.mission_completed = True
                return
                
            tx, ty, _ = self.trajectory[self.return_target_index]
            dx = tx - cx
            dy = ty - cy
            distance = math.hypot(dx, dy)

        # Calcul des angles vers la cible valide
        target_angle = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(target_angle - cyaw), math.cos(target_angle - cyaw))

        # AMÉLIORATION COMPORTEMENTALE : Loi de commande adaptative (Look-Ahead Speed)
        if abs(angle_error) > 0.45:  # Virage très serré (> 25°)
            # On stoppe la marche avant et on privilégie une rotation pure pour ne pas dévier du fil
            vz = self.RETURN_ANGULAR_SPEED if angle_error > 0 else -self.RETURN_ANGULAR_SPEED
            self._publish_velocity(0.0, vz)
            self.get_logger().info(f'↩️ Pivotement fort vers point #{self.return_target_index} | err={math.degrees(angle_error):.1f}°')
        
        elif abs(angle_error) > self.ANGLE_TOLERANCE:  # Ajustement léger de trajectoire
            # On avance doucement tout en appliquant la rotation pour fluidifier le mouvement
            vz = self.RETURN_ANGULAR_SPEED * 0.7 if angle_error > 0 else -self.RETURN_ANGULAR_SPEED * 0.7
            self._publish_velocity(self.RETURN_LINEAR_SPEED * 0.4, vz)
            self.get_logger().info(f'🔄 Courbe d\'ajustement vers point #{self.return_target_index}')
        
        else:  # Alignement parfait
            # Pleine vitesse en ligne droite
            vz = angle_error * 2.0
            self._publish_velocity(self.RETURN_LINEAR_SPEED, vz)
            self.get_logger().info(f'➡️ Ligne droite nominale | dist={distance:.2f}m')

    def _publish_velocity(self, vx, vz):
        req = Request()
        req.header.identity.id = 1
        req.header.identity.api_id = 1008  # Commande de déplacement matériel d'usine
        vel = {"x": vx, "y": 0.0, "z": vz}
        req.parameter = json.dumps(vel)
        self.cmd_pub.publish(req)

    def _emergency_stop(self):
        req = Request()
        req.header.identity.id = 1
        req.header.identity.api_id = 1008
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
        node.get_logger().info('Mission arrêtée manuellement.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()