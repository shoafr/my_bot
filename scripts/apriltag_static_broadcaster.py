#!/usr/bin/env python3
"""
apriltag_static_broadcaster.py (Mobile Camera Version)

Publishes static transforms for each tag, i.e. map -> tag_<id>,
based on the coordinates in your 'tag_positions.yaml' file.

Assumptions:
- The camera is on a moving robot, so 'map -> camera_link_optical' is NOT static.
- Each AprilTag is at a known, fixed pose in the map. We have (x, y, yaw) from your config.

Usage:
1) Place this file in 'my_bot/scripts/apriltag_static_broadcaster.py'
2) Make it executable & declare in your CMakeLists for installation
3) Provide a 'tag_positions.yaml' with your known tag <id> positions
4) Launch or run it. The tags appear in RViz under frames 'tag<id>', all parented by 'map'.
"""

import rclpy
from rclpy.node import Node
import yaml
import math

from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """
    Convert yaw/pitch/roll (in radians) to a Quaternion (REP-103).
    By default, pitch=roll=0 for planar rotation. This is enough for 2D ground setups.
    """
    q = Quaternion()
    half_roll  = roll  * 0.5
    half_pitch = pitch * 0.5
    half_yaw   = yaw   * 0.5

    sin_r = math.sin(half_roll)
    cos_r = math.cos(half_roll)
    sin_p = math.sin(half_pitch)
    cos_p = math.cos(half_pitch)
    sin_y = math.sin(half_yaw)
    cos_y = math.cos(half_yaw)

    # Z-Y-X rotation
    q.w = cos_r*cos_p*cos_y + sin_r*sin_p*sin_y
    q.x = sin_r*cos_p*cos_y - cos_r*sin_p*sin_y
    q.y = cos_r*sin_p*cos_y + sin_r*cos_p*sin_y
    q.z = cos_r*cos_p*sin_y - sin_r*sin_p*cos_y
    return q

class AprilTagStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('apriltag_static_broadcaster')

        # Parameter for YAML path
        self.declare_parameter('tag_positions_file', 'tag_positions.yaml')
        param_file = self.get_parameter('tag_positions_file').get_parameter_value().string_value

        # Load the YAML
        try:
            with open(param_file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load {param_file}: {e}")
            return

        tags = data.get('tags', None)
        if not tags:
            self.get_logger().error("No 'tags' dictionary found in YAML.")
            return

        # Create the StaticTransformBroadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        # For each AprilTag ID, publish map -> tag<id>
        for tag_id, pose in tags.items():
            x   = pose.get('x', 0.0)
            y   = pose.get('y', 0.0)
            yaw = pose.get('yaw', 0.0)

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"          # parent: map
            transform.child_frame_id = f"tag{tag_id}_static"  # child: e.g. tag5

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0

            q = euler_to_quaternion(yaw)  # roll=0, pitch=0
            transform.transform.rotation = q

            transforms.append(transform)
            self.get_logger().info(f"map->tag{tag_id}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("All static transforms (map->tags) published. Node will spin...")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStaticBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
