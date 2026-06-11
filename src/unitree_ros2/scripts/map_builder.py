#!/usr/bin/env python3

import os
import rclpy
import numpy as np
import open3d as o3d

from collections import deque
import bisect

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from scipy.spatial.transform import Rotation, Slerp


class MapBuilder(Node):

    def __init__(self):

        super().__init__('map_builder')

        # ==========================
        # Paramètres
        # ==========================
        self.declare_parameter('cloud_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('deskew', True)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('save_interval', 30.0)
        self.declare_parameter('map_dir', '~/unitree_ros2/map')

        self.map_dir = os.path.expanduser(self.get_parameter('map_dir').value)

        os.makedirs(self.map_dir, exist_ok=True)

        self.map_file = os.path.join(self.map_dir, 'global_map.pcd')

        # ==========================
        # Données
        # ==========================
        self.latest_odom = None

        # Buffer d'odométrie pour interpolation (timestamp en secondes)
        self.odom_buffer = deque(maxlen=500)

        self.global_cloud = o3d.geometry.PointCloud()

        self.scan_count = 0

        # ==========================
        # Subscribers
        # ==========================
        # Topics from parameters (compatible with Unilidar / Point-LIO)
        cloud_topic = self.get_parameter('cloud_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 10)

        # ==========================
        # Sauvegarde auto
        # ==========================
        save_interval = float(self.get_parameter('save_interval').value)
        self.create_timer(save_interval, self.save_map)

        self.get_logger().info(
            'Map Builder Started'
        )

    # =====================================
    # ODOM
    # =====================================
    def odom_callback(self, msg):
        # Store odometry in buffer for later interpolation
        header = msg.header
        t = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

        pose = msg.pose.pose
        tx = pose.position.x
        ty = pose.position.y
        tz = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # append as tuple (t, (tx,ty,tz), (qx,qy,qz,qw))
        self.odom_buffer.append((t, (tx, ty, tz), (qx, qy, qz, qw)))

        # keep latest_odom for compatibility
        self.latest_odom = msg

    # =====================================
    # CLOUD
    # =====================================
    def cloud_callback(self, msg):

        if len(self.odom_buffer) == 0:
            return

        # detect available fields (some Point-LIO clouds include a 't' field)
        available_fields = [f.name for f in msg.fields]
        use_time_field = 't' in available_fields or 'time' in available_fields
        time_field_name = 't' if 't' in available_fields else ('time' if 'time' in available_fields else None)

        # choose field names to read
        field_names = ("x", "y", "z")
        if use_time_field and time_field_name is not None:
            field_names = ("x", "y", "z", time_field_name)

        pts = []

        for p in point_cloud2.read_points(msg, field_names=field_names, skip_nans=True):
            if len(p) >= 4:
                pts.append((p[0], p[1], p[2], p[3]))
            else:
                pts.append((p[0], p[1], p[2], 0.0))

        if len(pts) == 0:
            return

        pts = np.array(pts)

        cloud_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        # Transform each point into world using odom buffer interpolation when possible
        world_points = np.empty((pts.shape[0], 3), dtype=np.float64)

        for i, p in enumerate(pts):
            px, py, pz, ptime = p
            point_time = cloud_time + float(ptime)

            tx, ty, tz, q = self.get_pose_at(point_time)

            R = Rotation.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()

            wp = (R @ np.array([px, py, pz])) + np.array([tx, ty, tz])
            world_points[i, :] = wp

        # create local cloud
        local_cloud = o3d.geometry.PointCloud()
        local_cloud.points = o3d.utility.Vector3dVector(world_points)

        # ==========================
        # Ajout à la carte
        # ==========================
        self.global_cloud += local_cloud

        # ==========================
        # Downsample voxel
        # ==========================
        voxel_size = float(self.get_parameter('voxel_size').value)
        self.global_cloud = self.global_cloud.voxel_down_sample(voxel_size=voxel_size)

        self.scan_count += 1

        if self.scan_count % 10 == 0:

            self.get_logger().info(
                f'Map points: '
                f'{len(self.global_cloud.points)}'
            )

    # =====================================
    # Pose interpolation (deskew)
    # =====================================
    def get_pose_at(self, t):
        """Return interpolated pose at time t (seconds).

        Returns: tx, ty, tz, (qx,qy,qz,qw)
        """
        buf = list(self.odom_buffer)
        if len(buf) == 0:
            return 0.0, 0.0, 0.0, (0.0, 0.0, 0.0, 1.0)

        times = [b[0] for b in buf]

        # clamp
        if t <= times[0]:
            _, trans, quat = buf[0]
            return trans[0], trans[1], trans[2], quat
        if t >= times[-1]:
            _, trans, quat = buf[-1]
            return trans[0], trans[1], trans[2], quat

        idx = bisect.bisect_left(times, t)
        if idx < len(times) and times[idx] == t:
            _, trans, quat = buf[idx]
            return trans[0], trans[1], trans[2], quat

        t0, trans0, quat0 = buf[idx - 1]
        t1, trans1, quat1 = buf[idx]

        if t1 == t0:
            return trans0[0], trans0[1], trans0[2], quat0

        alpha = (t - t0) / (t1 - t0)

        tx = trans0[0] * (1 - alpha) + trans1[0] * alpha
        ty = trans0[1] * (1 - alpha) + trans1[1] * alpha
        tz = trans0[2] * (1 - alpha) + trans1[2] * alpha

        # slerp for rotation
        r0 = [quat0[0], quat0[1], quat0[2], quat0[3]]
        r1 = [quat1[0], quat1[1], quat1[2], quat1[3]]
        rots = Rotation.from_quat([r0, r1])
        slerp = Slerp([t0, t1], rots)
        r_interp = slerp([t])[0]
        q_interp = r_interp.as_quat()

        return tx, ty, tz, (q_interp[0], q_interp[1], q_interp[2], q_interp[3])

    # =====================================
    # SAVE MAP
    # =====================================
    def save_map(self):

        if len(self.global_cloud.points) == 0:
            return

        o3d.io.write_point_cloud(
            self.map_file,
            self.global_cloud
        )

        self.get_logger().info(
            f'Carte sauvegardée : '
            f'{self.map_file}'
        )


# ==========================================
# MAIN
# ==========================================
def main():

    rclpy.init()

    node = MapBuilder()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print('\nArrêt demandé...')

    finally:

        node.save_map()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()