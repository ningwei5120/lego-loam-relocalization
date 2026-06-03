#!/usr/bin/env python3
"""
Publish an initial pose to /initialpose topic.
Supports command-line args or interactive input.

Usage:
  # Default: map origin (0, 0, 0, yaw=0)
  rosrun lego_relocalization publish_initialpose.py

  # Specify pose via args
  rosrun lego_relocalization publish_initialpose.py --x 10.0 --y 20.0 --z 0.0 --yaw 1.57

  # Interactive mode (prompts for input)
  rosrun lego_relocalization publish_initialpose.py --interactive
"""

import argparse
import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header


def parse_args():
    parser = argparse.ArgumentParser(description="Publish /initialpose for relocalization")
    parser.add_argument("--x", type=float, default=0.0, help="X position (default: 0.0)")
    parser.add_argument("--y", type=float, default=0.0, help="Y position (default: 0.0)")
    parser.add_argument("--z", type=float, default=0.0, help="Z position (default: 0.0)")
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="Yaw angle in radians (default: 0.0). Use --yaw-deg for degrees.")
    parser.add_argument("--yaw-deg", type=float, default=None,
                        help="Yaw angle in degrees (overrides --yaw)")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll in radians (default: 0.0)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch in radians (default: 0.0)")
    parser.add_argument("--frame-id", type=str, default="map", help="Frame ID (default: map)")
    parser.add_argument("--topic", type=str, default="/initialpose", help="Topic name")
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="Interactive mode: prompt for pose input")
    return parser.parse_args()


def interactive_input():
    """Prompt user for pose values."""
    print("\n=== Manual Initial Pose Input ===")
    print("Enter pose values (press Enter to use default in brackets):")
    try:
        x = float(input("  X position [0.0]: ") or "0.0")
    except ValueError:
        x = 0.0
    try:
        y = float(input("  Y position [0.0]: ") or "0.0")
    except ValueError:
        y = 0.0
    try:
        z = float(input("  Z position [0.0]: ") or "0.0")
    except ValueError:
        z = 0.0
    try:
        yaw_deg = input("  Yaw angle in degrees [0.0]: ") or "0.0"
        yaw = math.radians(float(yaw_deg))
    except ValueError:
        yaw = 0.0
    return x, y, z, yaw


def build_pose_msg(x, y, z, roll, pitch, yaw, frame_id):
    """Build PoseWithCovarianceStamped message."""
    msg = PoseWithCovarianceStamped()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z

    # Quaternion from roll-pitch-yaw (ZYX order)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
    msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy

    # Default covariance: moderate uncertainty
    cov = [0.25, 0, 0, 0, 0, 0,
           0, 0.25, 0, 0, 0, 0,
           0, 0, 0.25, 0, 0, 0,
           0, 0, 0, 0.0685, 0, 0,
           0, 0, 0, 0, 0.0685, 0,
           0, 0, 0, 0, 0, 0.0685]
    msg.pose.covariance = cov

    return msg


def main():
    args = parse_args()

    rospy.init_node("publish_initialpose", anonymous=True)
    pub = rospy.Publisher(args.topic, PoseWithCovarianceStamped, queue_size=1)

    # Wait for publisher to connect
    rospy.sleep(0.5)

    if args.interactive:
        x, y, z, yaw = interactive_input()
        roll, pitch = 0.0, 0.0
    else:
        x, y, z = args.x, args.y, args.z
        roll, pitch = args.roll, args.pitch
        yaw = math.radians(args.yaw_deg) if args.yaw_deg is not None else args.yaw

    msg = build_pose_msg(x, y, z, roll, pitch, yaw, args.frame_id)
    pub.publish(msg)

    print(f"\nPublished /initialpose:")
    print(f"  Position:  [{x:.3f}, {y:.3f}, {z:.3f}]")
    print(f"  RPY (deg): [{math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}]")
    print(f"  Frame:     {args.frame_id}")
    print(f"\nTip: In RViz, you can also use '2D Pose Estimate' tool to click & drag.")


if __name__ == "__main__":
    main()
