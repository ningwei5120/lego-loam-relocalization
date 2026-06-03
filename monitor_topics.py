#!/usr/bin/env python3
import sys
sys.stdout = open(sys.stdout.fileno(), mode='w', encoding='utf-8', buffering=1)
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

counts = {
    'velodyne_points': 0,
    'segmented_cloud': 0,
    'laser_odom_to_init': 0,
    'aft_mapped_to_init': 0,
    'laser_cloud_corner_last': 0,
    'laser_cloud_surf_last': 0,
}

def make_cb(name):
    def cb(msg):
        counts[name] += 1
    return cb

rospy.init_node('topic_monitor')
rospy.Subscriber('/velodyne_points', PointCloud2, make_cb('velodyne_points'))
rospy.Subscriber('/segmented_cloud', PointCloud2, make_cb('segmented_cloud'))
rospy.Subscriber('/laser_odom_to_init', Odometry, make_cb('laser_odom_to_init'))
rospy.Subscriber('/aft_mapped_to_init', Odometry, make_cb('aft_mapped_to_init'))
rospy.Subscriber('/laser_cloud_corner_last', PointCloud2, make_cb('laser_cloud_corner_last'))
rospy.Subscriber('/laser_cloud_surf_last', PointCloud2, make_cb('laser_cloud_surf_last'))

try:
    rospy.sleep(30)
except rospy.ROSInterruptException:
    pass

for k, v in counts.items():
    print(f"{k}: {v}", flush=True)
