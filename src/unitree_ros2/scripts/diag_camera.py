#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData

class DiagCamera(Node):
    def __init__(self):
        super().__init__('diag_camera')
        self.sub = self.create_subscription(
            Go2FrontVideoData,
            '/frontvideostream',
            self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(
            f'720p={len(msg.video720p)} bytes | '
            f'360p={len(msg.video360p)} bytes | '
            f'180p={len(msg.video180p)} bytes | '
            f'time={msg.time_frame}'
        )
        # Affiche les premiers octets pour identifier le format
        if msg.video360p:
            header = bytes(msg.video360p[:8])
            self.get_logger().info(f'Header 360p: {header.hex()}')

def main():
    rclpy.init()
    rclpy.spin(DiagCamera())

if __name__ == '__main__':
    main()