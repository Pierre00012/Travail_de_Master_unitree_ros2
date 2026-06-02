#!/usr/bin/env python3
"""Odom -> TF broadcaster

Ce nœud écoute le topic d'odométrie `/utlidar/robot_odom` (nav_msgs/Odometry)
et publie un transform TF `odom -> base_link` en copiant la position
et l'orientation du message d'odométrie.

Note importante : ce fichier doit être lancé AVANT le lancement de
RTAB-Map afin que le transform `odom -> base_link` soit disponible pour
le processus de mapping/localisation. python3 odom_to_tf.py
"""

import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTF(Node):

    def __init__(self):

        super().__init__('odom_tf')

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.callback,
            10
        )

        print("ODOM TF STARTED")

    def callback(self, msg):

        t = TransformStamped()

        # IMPORTANT
        t.header = msg.header

        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

        print("TF SENT")


def main():

    rclpy.init()

    node = OdomTF()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()