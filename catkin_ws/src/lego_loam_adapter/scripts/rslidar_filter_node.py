#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import struct
import math

class RslidarFilterNode:
    def __init__(self):
        rospy.init_node('rslidar_filter_node')
        self.sub = rospy.Subscriber('/rslidar_points', PointCloud2, self.callback, queue_size=2)
        self.pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=2)
        rospy.loginfo("RslidarFilterNode started: /rslidar_points -> /velodyne_points")

    def callback(self, msg):
        # Find x,y,z offsets
        fields = {f.name: f for f in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            rospy.logerr("Missing x/y/z fields in PointCloud2")
            return
        x_off = fields['x'].offset
        y_off = fields['y'].offset
        z_off = fields['z'].offset

        new_data = bytearray()
        point_count = msg.width * msg.height
        for i in range(point_count):
            base = i * msg.point_step
            x = struct.unpack_from('<f', msg.data, base + x_off)[0]
            y = struct.unpack_from('<f', msg.data, base + y_off)[0]
            z = struct.unpack_from('<f', msg.data, base + z_off)[0]
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                new_data.extend(msg.data[base:base + msg.point_step])

        if len(new_data) == 0:
            rospy.logwarn_throttle(5.0, "Empty cloud after filtering NaN")
            return

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.header.frame_id = "camera_init"
        new_msg.height = 1
        new_msg.width = len(new_data) // msg.point_step
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = len(new_data)
        new_msg.is_dense = True
        new_msg.data = bytes(new_data)
        self.pub.publish(new_msg)

if __name__ == '__main__':
    node = RslidarFilterNode()
    rospy.spin()
