#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from unitree_go.msg import SportModeState


class RobotStateMonitor(Node):

    def __init__(self):

        super().__init__('robot_state_monitor')

        self.sub = self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self.callback,
            10
        )

        self.get_logger().info("Robot State Monitor Started")

    def callback(self, msg):

        vx = msg.velocity[0]

        self.get_logger().info(
            f"Vitesse X: {vx:.2f} | Obstacle interne: {msg.range_obstacle}"
        )


def main():

    rclpy.init()
    node = RobotStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()