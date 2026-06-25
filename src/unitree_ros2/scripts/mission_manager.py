#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from unitree_api.msg import Request
import json
import time  # Utilisé pour time.monotonic()

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ===== CONFIGURATION =====
        self.MISSION_DURATION = 60.0  # Durée de la mission en secondes
        self.FREQUENCY = 1.0           # Fréquence du monitoring (1 Hz)

        # ===== ETAT INTERNE IMMUNISÉ =====
        # time.monotonic() renvoie le temps brut du CPU, insensible aux sauts de temps
        self.start_time_mono = time.monotonic()
        self.mission_completed = False

        # ===== CONFIGURATION ROS 2 =====
        # 1. Action Client vers Nav2 pour gérer le retour au point de départ (0,0)
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 2. Publisher pour forcer l'arrêt d'urgence des moteurs si nécessaire
        self.cmd_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 3. Timer de monitoring de la mission (s'exécute toutes les secondes)
        self.monitoring_timer = self.create_timer(self.FREQUENCY, self.mission_monitoring_loop)

        self.get_logger().info('======================================================')
        self.get_logger().info('🚀 SUPERVISEUR : Mission lancée.')
        self.get_logger().info('👉 Cartographie active (RTAB-Map).')
        self.get_logger().info('👉 Évitement d\'obstacles actif (obstacle_avoidance).')
        self.get_logger().info(f'⏱️ Durée de la phase d\'exploration : {self.MISSION_DURATION:.1f} secondes.')
        self.get_logger().info('======================================================')

    def mission_monitoring_loop(self):
        """Boucle de surveillance exécutée à 1 Hz (Protégée contre les sauts de temps)"""
        if self.mission_completed:
            return

        # Calcul du temps écoulé de manière linéaire et sécurisée
        elapsed_time = time.monotonic() - self.start_time_mono
        remaining_time = max(0.0, self.MISSION_DURATION - elapsed_time)

        # Log de suivi toutes les 10 secondes (sans risque de doublon ou de saut)
        if int(elapsed_time) > 0 and int(elapsed_time) % 10 == 0:
            if not hasattr(self, '_last_log_sec') or self._last_log_sec != int(elapsed_time):
                self.get_logger().info(
                    f'⏱️ Statut Mission : Temps écoulé = {elapsed_time:.1f}s | Temps restant = {remaining_time:.1f}s'
                )
                self._last_log_sec = int(elapsed_time)

        # Condition de déclenchement stricte et inévitable
        if elapsed_time >= self.MISSION_DURATION:
            self.get_logger().warn('⏰ CHRONOMÈTRE ÉCOULÉ ! Déclenchement de la phase de retour à la base.')
            self.monitoring_timer.cancel()  # On coupe le timer de monitoring pour de bon
            self.trigger_return_to_base()

    def trigger_return_to_base(self):
        """Prend le contrôle du robot et ordonne le retour au repère d'origine"""
        self.get_logger().info('⏳ Attente de la synchronisation avec le serveur Nav2...')
        
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Serveur Nav2 indisponible ! Impossible d\'ordonner le retour autonome.')
            self._emergency_stop()
            return

        # Configuration de l'objectif Nav2 géométrique (Retour à l'origine de la Map)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Le stamp du message reste sur l'horloge ROS 2 pour Nav2

        # Coordonnées initiales (X=0.0, Y=0.0, Orientation Neutre)
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().warn('🎯 Objectif (0.0, 0.0) envoyé à Nav2. Calcul du chemin optimal de retour en cours...')
        
        # Envoi asynchrone de l'objectif
        self._send_goal_future = self._nav2_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._nav2_feedback_callback
        )
        self._send_goal_future.add_done_callback(self._nav2_goal_response_callback)

    def _nav2_goal_response_callback(self, future):
        """Vérifie si Nav2 a accepté ou refusé l'ordre de retour"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 a REFUSÉ l\'objectif de retour au point de départ !')
            self._emergency_stop()
            return

        self.get_logger().info('✅ Objectif de retour ACCEPTÉ par Nav2. Le robot fait demi-tour.')
        self.mission_completed = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._nav2_result_callback)

    def _nav2_feedback_callback(self, feedback_msg):
        """Feedback en direct de la trajectoire de retour"""
        pass

    def _nav2_result_callback(self, future):
        """Exécuté lorsque le robot est physiquement arrivé à destination"""
        status = future.result().status
        if status == 4:  # Status Succeeded dans l'API Nav2
            self.get_logger().info('======================================================')
            self.get_logger().info('🥳 MISSION RÉUSSIE : Le robot est revenu à sa position initiale !')
            self.get_logger().info('======================================================')
        else:
            self.get_logger().error(f'⚠️ Échec de la trajectoire de retour autonome. Code statut Nav2 : {status}')
        self._emergency_stop()

    def _emergency_stop(self):
        """Sécurité : Force l'immobilisation des moteurs du Unitree Go2"""
        self.get_logger().info('🛑 Immobilisation des moteurs.')
        msg = Request()
        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008  # SDK Sport Unitree (vitesse)
        msg.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Superviseur arrêté par l\'utilisateur.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()