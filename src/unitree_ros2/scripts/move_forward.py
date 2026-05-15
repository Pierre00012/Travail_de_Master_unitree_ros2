#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
import json

class MoveForward(Node):

    def __init__(self):
        super().__init__('move_forward')

        self.pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

        # 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.send_cmd
        )

        self.get_logger().info(
            "Go2 moving forward..."
        )

    def send_cmd(self):

        msg = Request()

        msg.header.identity.id = 1
        msg.header.identity.api_id = 1008

        velocity = {
            "x": 0.35,
            "y": 0.0,
            "z": 0.0
        }

        msg.parameter = json.dumps(velocity)

        self.pub.publish(msg)


def main():

    rclpy.init()

    node = MoveForward()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()