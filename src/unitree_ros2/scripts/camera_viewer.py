#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.sub = self.create_subscription(
            Go2FrontVideoData,
            '/frontvideostream',
            self.cb, 10)
        self.get_logger().info('Camera viewer started')

    def cb(self, msg):
        # Utilise la 360p pour ne pas surcharger
        data = bytes(msg.video360p)
        if not data:
            return

        # Décode le flux JPEG/H264
        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow('GO2 Front Camera', frame)
            cv2.waitKey(1)
        else:
            self.get_logger().warn('Frame non decodable')

def main():
    rclpy.init()
    rclpy.spin(CameraViewer())

if __name__ == '__main__':
    main()