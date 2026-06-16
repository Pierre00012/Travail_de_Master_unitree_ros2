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

    # ============================================================
    # CONSTANTES DE CONFIGURATION (PARAMÉTRAGE MASTER)
    # ============================================================
    # Seuils de sécurité (en mètres)
    OBSTACLE_DISTANCE = 1.3       # Zone d'influence des forces répulsives
    CRITICAL_DISTANCE = 0.65      # Seuil de déclenchement du recul d'urgence
    MIN_LIDAR_DISTANCE = 0.45     # Filtrage du bruit proche châssis

    # Boîte de coupure LiDAR (Bounding Box de sécurité)
    LIDAR_MAX_X = -0.35
    LIDAR_MAX_Y = 0.45
    LIDAR_BLIND_ZONE_X = 0.5
    LIDAR_BLIND_ZONE_Y = 0.45

    ESCAPE_PHASE1_DURATION = 4.0  # Temps de recul (en secondes)
    ESCAPE_PHASE2_DURATION = 2.0  # Temps de rotation (en secondes)
    ESCAPE_TOTAL_TIMEOUT = 15.0
    
    # ÉLÉVATION (AMÉLIORATION ANGLE MORT) : Protection accrue du dos du robot
    LIDAR_MIN_Z = -0.05           # Détection des obstacles bas au sol
    LIDAR_MAX_Z = 0.45            # Protection contre les rebords de bureaux / tables hautes

    STARTUP_DELAY = 10.0

    # Dynamique des vitesses (Lissées pour le SLAM)
    SPEED_MAX_FORWARD = 0.3       # Vitesse de croisière nominale
    SPEED_MIN_FORWARD = 0.12      # Vitesse d'approche prudente
    SPEED_BACKWARD = -0.18        # Recul lent pour éviter de décrocher l'ICP
    SPEED_MAX_TURN = 0.6          # Vitesse de rotation maximale lissée

    # Gains du Champ de Potentiel Virtuel (VPF)
    K_REPULSIVE = 0.4             # Force de rejet des obstacles latéraux

    # Profil d'accélération (Rampes de lissage temporel)
    # Plus la valeur est petite, plus les transitions de vitesse sont douces
    RAMP_ALPHA_VX = 0.12          # Lissage de l'accélération linéaire
    RAMP_ALPHA_VZ = 0.18          # Lissage de l'accélération angulaire

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ===== ÉTAT DES DISTANCES =====
        self.front_dist = 999.0
        self.left_repulsion = 0.0
        self.right_repulsion = 0.0

        # Commandes cibles (Calculées par l'algorithme)
        self.vx_target = 0.0
        self.vz_target = 0.0

        # Commandes effectives (Issues des rampes de lissage pour stabiliser le LiDAR)
        self.vx_current = 0.0
        self.vz_current = 0.0

        # Machine d'état
        self.state = 'INIT'
        self.start_time = time.time()
        self.escape_start_time = 0.0
        self.last_log_time = 0.0

        # ===== CONFIGURATION ROS 2 =====
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.lidar_callback, 10)

        self.pub = self.create_publisher(
            Request, '/api/sport/request', 10)

        # Fréquence de contrôle fixée à 20 Hz (Toutes les 50ms)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('✅ Nœud d\'évitement optimisé pour SLAM initialisé.')

    def _create_request_msg(self):
        msg = Request()
        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008
        return msg
    
    def _publish_velocity(self, vx, vz):
        """Applique les rampes d'accélération et publie les vitesses réelles"""
        # Application mathématique du filtre passe-bas (Rampes de lissage)
        self.vx_current = self.vx_current + self.RAMP_ALPHA_VX * (vx - self.vx_current)
        self.vz_current = self.vz_current + self.RAMP_ALPHA_VZ * (vz - self.vz_current)

        # Envoi au SDK Sport Unitree
        msg = self._create_request_msg()
        velocity = {"x": self.vx_current, "y": 0.0, "z": self.vz_current}
        msg.parameter = json.dumps(velocity)
        self.pub.publish(msg)

    # ============================================================
    # PIPELINE DE PERCEPTION : CHAMPS DE POTENTIELS VIRTUELS
    # ============================================================
    def lidar_callback(self, msg):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        min_front_dist = 999.0
        accumulated_left_force = 0.0
        accumulated_right_force = 0.0

        for p in points:
            x, y, z = p

            # 1. Filtres d'exclusion géométriques
            if x > self.LIDAR_MAX_X or abs(y) > self.LIDAR_MAX_Y:
                continue
            if abs(x) < self.LIDAR_BLIND_ZONE_X and abs(y) < self.LIDAR_BLIND_ZONE_Y:
                continue
            if z < self.LIDAR_MIN_Z or z > self.LIDAR_MAX_Z:
                continue

            # Distance Euclidienne du point
            d = math.sqrt(x*x + y*y + z*z)
            if d < self.MIN_LIDAR_DISTANCE:
                continue

            # 2. Analyse de la trajectoire axiale (Zone Centrale d'impact)
            if abs(y) < 0.10:  # Couloir central resserré pour éviter les faux arrêts
                min_front_dist = min(min_front_dist, d)
            
            # 3. Calcul des Forces Répulsives Continues (Algorithme VPF)
            if d < self.OBSTACLE_DISTANCE:
                # La force augmente de manière quadratique à mesure que l'obstacle approche
                force = (1.0 / d) - (1.0 / self.OBSTACLE_DISTANCE)
                
                if y > 0:  # Obstacle à gauche -> génère une force poussant vers la droite
                    accumulated_left_force += force * (abs(y) / d)
                else:      # Obstacle à droite -> génère une force poussant vers la gauche
                    accumulated_right_force += force * (abs(y) / d)

        # Sauvegarde de l'état
        self.front_dist = min_front_dist
        self.left_repulsion = accumulated_left_force
        self.right_repulsion = accumulated_right_force

        # Télémétrie de contrôle (1 Hz)
        if time.time() - self.last_log_time > 1.0:
            self.get_logger().info(
                f' Perception -> Front: {self.front_dist:.2f}m | F_gauche: {self.left_repulsion:.2f} | F_droite: {self.right_repulsion:.2f}'
            )
            self.last_log_time = time.time()

    # ============================================================
    # BOUCLE DE DÉCISION (MACHINE D'ÉTAT SÉCURISÉE)
    # ============================================================
    def control_loop(self):
        elapsed = time.time() - self.start_time

        # État INITIALISATION
        if self.state == 'INIT':
            if elapsed < self.STARTUP_DELAY:
                return
            else:
                self.state = 'RUNNING'
                self.get_logger().info('✅ Système de survie en mode NOMINAL.')

        # État MANŒUVRE DE DÉGAGEMENT
        if self.state == 'ESCAPE':
            self._execute_escape_sequence()
            return

        # État RUNNING (Navigation Autonome Intelligente)
        self._navigate_potentials()

    def _navigate_potentials(self):
        """Calcule des trajectoires fluides basées sur l'équilibre des forces"""
        
        # CAS 1 : Obstacle critique imminent -> Recul immédiat (sans secousse)
        if self.front_dist <= self.CRITICAL_DISTANCE:
            self.get_logger().warn(f'🚨 Obstacle critique détecté ({self.front_dist:.2f}m) -> Transition ESCAPE')
            self.state = 'ESCAPE'
            self.escape_start_time = time.time()
            self._publish_velocity(0.0, 0.0)
            return

        # CAS 2 : Trajectoire nominale ou évitement fluide
        # Calcul de la vitesse linéaire proportionnelle à l'espace libre devant
        if self.front_dist < self.OBSTACLE_DISTANCE:
            # Interpolation linéaire de la vitesse pour ralentir à l'approche du mur
            ratio = (self.front_dist - self.CRITICAL_DISTANCE) / (self.OBSTACLE_DISTANCE - self.CRITICAL_DISTANCE)
            ratio = max(0.0, min(1.0, ratio))
            self.vx_target = self.SPEED_MIN_FORWARD + ratio * (self.SPEED_MAX_FORWARD - self.SPEED_MIN_FORWARD)
        else:
            self.vx_target = self.SPEED_MAX_FORWARD

        # Calcul de la vitesse angulaire (Équilibre des forces vectorielles)
        # Si F_droite > F_gauche, la résultante vz est positive (tourne à gauche)
        # Si F_gauche > F_droite, la résultante vz est négative (tourne à droite)
        raw_repulsion = (self.right_repulsion - self.left_repulsion) * self.K_REPULSIVE
        
        # Centrage automatique en ligne droite : Si les forces sont nulles, pas de biais arbitraire !
        self.vz_target = max(-self.SPEED_MAX_TURN, min(self.SPEED_MAX_TURN, raw_repulsion))

        # Envoi des consignes à la fonction de lissage
        self._publish_velocity(self.vx_target, self.vz_target)

    def _execute_escape_sequence(self):
        """Séquence de secours lissée pour préserver l'alignement de la carte SLAM"""
        elapsed_escape = time.time() - self.escape_start_time

        # Phase 1 : Recul progressif
        if elapsed_escape < self.ESCAPE_PHASE1_DURATION:
            self._publish_velocity(self.SPEED_BACKWARD, 0.0)
            
        # Phase 2 : Pivotement contrôlé du côté le plus dégagé
        elif elapsed_escape < (self.ESCAPE_PHASE1_DURATION + self.ESCAPE_PHASE2_DURATION):
            # Détermination de la rotation optimale selon l'historique des forces
            direction = 1.0 if self.right_repulsion <= self.left_repulsion else -1.0
            self._publish_velocity(0.0, direction * self.SPEED_MAX_TURN)
            
        # Phase 3 : Sortie de secours sécurisée
        else:
            if self.front_dist > self.OBSTACLE_DISTANCE:
                self.get_logger().info('✅ Zone dégagée. Reprise de l\'exploration.')
                self.state = 'RUNNING'
            else:
                # Si toujours bloqué, on continue de pivoter sur place doucement
                direction = 1.0 if self.right_repulsion <= self.left_repulsion else -1.0
                self._publish_velocity(0.0, direction * (self.SPEED_MAX_TURN * 0.6))

            if elapsed_escape > self.ESCAPE_TOTAL_TIMEOUT:
                self.get_logger().error('⏱️ Timeout de la manœuvre d\'évacuation.')
                self.state = 'RUNNING'

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt manuel demandé.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()