#!/usr/bin/env python3
"""
Scan Context Auto-Initializer Node (v3)
---------------------------------------
Builds SC database in ORIGINAL coordinate system (rich z-variation)
and converts matched pose to SWAPPED coordinate system before publishing.
"""

import os
import sys
import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_context import ScanContextManager, build_database_from_pcd


def swap_pose_xyz(x_orig, y_orig, z_orig):
    """
    Coordinate swap: (x,y,z)_orig -> (z,x,y)_swapped
    This matches the swap applied in ndt_icp_relocalize.cpp loadGlobalMap().
    """
    return z_orig, x_orig, y_orig


class ScanContextInitializer:
    def __init__(self):
        rospy.init_node('scan_context_initializer')

        self.pcd_map_path = rospy.get_param('~pcd_map_path',
                                            '/home/hong/lego/map_results/finalCloud.pcd')
        self.db_path = rospy.get_param('~db_path',
                                       '/home/hong/lego/map_results/scancontext_db.npz')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/velodyne_points')
        self.odom_topic = rospy.get_param('~odom_topic', '/relocalization/odometry')
        self.initialpose_topic = rospy.get_param('~initialpose_topic', '/initialpose')
        self.map_frame = rospy.get_param('~map_frame', 'map')

        self.sc_num_rings = rospy.get_param('~sc_num_rings', 20)
        self.sc_num_sectors = rospy.get_param('~sc_num_sectors', 60)
        self.sc_max_len = rospy.get_param('~sc_max_len', 80.0)
        self.grid_res = rospy.get_param('~grid_res', 5.0)
        self.local_radius = rospy.get_param('~local_radius', 30.0)
        self.query_k = rospy.get_param('~query_k', 5)
        self.min_points = rospy.get_param('~min_points', 500)
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)
        self.cooldown_sec = rospy.get_param('~cooldown_sec', 3.0)
        self.max_sc_distance = rospy.get_param('~max_sc_distance', 600.0)
        self.tracking_timeout = rospy.get_param('~tracking_timeout', 2.0)

        self.latest_cloud = None
        self.last_publish_time = rospy.Time(0)
        self.last_odom_time = rospy.Time(0)
        self.is_tracking = False
        self.sc_manager = None

        self._init_database()

        self.sub_cloud = rospy.Subscriber(self.lidar_topic, PointCloud2,
                                          self._cloud_callback, queue_size=2)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry,
                                         self._odom_callback, queue_size=10)
        self.pub_initialpose = rospy.Publisher(self.initialpose_topic,
                                               PoseWithCovarianceStamped, queue_size=1)

        rospy.loginfo("[SC-Initializer] Ready. Will auto-publish /initialpose when needed.")

    def _init_database(self):
        if os.path.exists(self.db_path):
            rospy.loginfo(f"[SC-Initializer] Loading DB: {self.db_path}")
            self.sc_manager = ScanContextManager(
                num_rings=self.sc_num_rings,
                num_sectors=self.sc_num_sectors,
                max_len=self.sc_max_len
            )
            self.sc_manager.load_database(self.db_path)
            rospy.loginfo(f"[SC-Initializer] Loaded {len(self.sc_manager.poses)} places")
        else:
            rospy.loginfo(f"[SC-Initializer] Building DB from: {self.pcd_map_path}")
            self.sc_manager = build_database_from_pcd(
                self.pcd_map_path,
                grid_res=self.grid_res,
                local_radius=self.local_radius,
                sc_manager=ScanContextManager(
                    num_rings=self.sc_num_rings,
                    num_sectors=self.sc_num_sectors,
                    max_len=self.sc_max_len
                ),
                min_points=self.min_points
            )
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            self.sc_manager.save_database(self.db_path)
            rospy.loginfo(f"[SC-Initializer] Saved DB to: {self.db_path}")

    def _cloud_callback(self, msg):
        pts = self._parse_pointcloud2(msg)
        if pts is not None and len(pts) > 0:
            self.latest_cloud = pts

    def _odom_callback(self, msg):
        self.last_odom_time = rospy.Time.now()
        self.is_tracking = True

    @staticmethod
    def _parse_pointcloud2(msg):
        if msg.width == 0 or msg.height == 0:
            return None
        fields = {f.name: f.offset for f in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            rospy.logwarn_throttle(5.0, "[SC-Initializer] Missing x/y/z fields")
            return None
        x_off, y_off, z_off = fields['x'], fields['y'], fields['z']
        point_step = msg.point_step
        data = msg.data
        num_pts = msg.width * msg.height
        pts = np.zeros((num_pts, 3), dtype=np.float32)
        for i in range(num_pts):
            base = i * point_step
            pts[i, 0] = struct.unpack_from('<f', data, base + x_off)[0]
            pts[i, 1] = struct.unpack_from('<f', data, base + y_off)[0]
            pts[i, 2] = struct.unpack_from('<f', data, base + z_off)[0]
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]

    def _check_tracking_active(self):
        if self.last_odom_time.is_zero():
            return False
        return (rospy.Time.now() - self.last_odom_time).to_sec() < self.tracking_timeout

    def _query_and_publish(self):
        if self._check_tracking_active():
            return

        if self.latest_cloud is None:
            return

        pts = self.latest_cloud
        if len(pts) < self.min_points:
            rospy.logwarn_throttle(5.0, "[SC-Initializer] Too few points, skipping")
            return

        # Both query cloud and DB are in ORIGINAL coordinate system
        results = self.sc_manager.query(pts, k=self.query_k)
        if not results:
            return

        best_dist, best_shift, best_pose = results[0]

        if best_dist > self.max_sc_distance:
            rospy.logwarn_throttle(5.0,
                f"[SC-Initializer] Best match distance too large: {best_dist:.1f}, rejecting")
            return

        # Pose in ORIGINAL coordinate system
        x_orig, y_orig, z_orig = best_pose[0], best_pose[1], 0.0
        yaw_shift = best_shift * (2 * math.pi / self.sc_manager.num_sectors)
        yaw_orig = math.atan2(math.sin(yaw_shift), math.cos(yaw_shift))

        # Convert to SWAPPED coordinate system used by ndt_icp_relocalize
        # Swap: (x,y,z)_orig -> (z,x,y)_swapped
        x_s, y_s, z_s = swap_pose_xyz(x_orig, y_orig, z_orig)

        # For yaw: original yaw is rotation around original Z axis.
        # After swap, original Z becomes new X. So original yaw becomes rotation around new X (roll).
        # But the relocalization node's initial pose expects yaw in the new horizontal plane.
        # The new horizontal plane is spanned by new X (orig Z) and new Y (orig X).
        # A rotation around original Z means the vehicle turns left/right in the original XY plane.
        # In the swapped system, original XY plane becomes new YZ plane.
        # So the equivalent rotation in swapped horizontal plane (new XY) is different.
        #
        # For simplicity, we map the original yaw to the new yaw by recognizing that
        # a 90-degree rotation in original XY corresponds to a 90-degree rotation in new YZ.
        # The new XY plane uses new X=orig Z and new Y=orig X.
        # If we look from above in the new system (down new Z=orig Y), new X points to orig Z,
        # and new Y points to orig X.
        # An original yaw of 0 means forward is +orig X = +new Y.
        # An original yaw of 90deg means forward is +orig Y = +new Z.
        # In the new horizontal plane (new XY), forward along +new Y corresponds to yaw=0.
        # Forward along +new X (orig Z) corresponds to yaw=90deg.
        # So: yaw_new = 90deg - yaw_orig (in radians: pi/2 - yaw_orig)
        yaw_s = math.pi / 2.0 - yaw_orig
        yaw_s = math.atan2(math.sin(yaw_s), math.cos(yaw_s))

        rospy.loginfo(f"[SC-Initializer] Match: dist={best_dist:.1f}, "
                      f"orig=({x_orig:.1f},{y_orig:.1f},yaw={math.degrees(yaw_orig):.1f}), "
                      f"swapped=({x_s:.1f},{y_s:.1f},yaw={math.degrees(yaw_s):.1f})")

        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x_s
        msg.pose.pose.position.y = y_s
        msg.pose.pose.position.z = z_s
        qz = math.sin(yaw_s / 2.0)
        qw = math.cos(yaw_s / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.25, 0, 0, 0,
               0, 0, 0, 0.0685, 0, 0,
               0, 0, 0, 0, 0.0685, 0,
               0, 0, 0, 0, 0, 0.0685]
        msg.pose.covariance = cov

        self.pub_initialpose.publish(msg)
        self.last_publish_time = rospy.Time.now()
        rospy.loginfo("[SC-Initializer] Published /initialpose")

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_publish_time).to_sec() > self.cooldown_sec:
                self._query_and_publish()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = ScanContextInitializer()
        node.run()
    except rospy.ROSInterruptException:
        pass
