#!/usr/bin/env python3
"""
apriltag_static_broadcaster.py (Enhanced)

Publishes static transforms for each tag, i.e. map -> tag<ID>_static,
with full (x, y, z, roll, pitch, yaw) support from 'tag_positions.yaml'.

Usage:
1) Place in 'my_bot/scripts/apriltag_static_broadcaster.py'
2) Make it executable, update CMakeLists if needed
3) Provide 'tag_positions.yaml' with each tag's (x,y,z,roll,pitch,yaw)
4) Launch or run it. Frames appear under 'tag<ID>_static' in TF.

Example YAML:
tags:
  5:
    x: 1.14
    y: -0.81
    z: 0.2
    roll: 0.0
    pitch: 1.57
    yaw: 3.14
  7:
    x: 3.0
    y: 2.0
    z: 0.5
    roll: 0.0
    pitch: 0.0
    yaw: 1.57
"""

import rclpy
from rclpy.node import Node
import yaml
import math

from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert roll/pitch/yaw (in radians) to a ROS geometry_msgs/Quaternion (REP-103).
    'Z-Y-X' rotation order by default: 
      roll  around X
      pitch around Y
      yaw   around Z
    """
    half_roll  = roll  * 0.5
    half_pitch = pitch * 0.5
    half_yaw   = yaw   * 0.5

    sin_r = math.sin(half_roll)
    cos_r = math.cos(half_roll)
    sin_p = math.sin(half_pitch)
    cos_p = math.cos(half_pitch)
    sin_y = math.sin(half_yaw)
    cos_y = math.cos(half_yaw)

    q = Quaternion()
    # Z-Y-X rotation
    q.w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    q.x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    q.y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    q.z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
    return q

class AprilTagStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('apriltag_static_broadcaster')

        # Parameter for the YAML file path
        self.declare_parameter('tag_positions_file', 'tag_positions.yaml')
        param_file = self.get_parameter('tag_positions_file').get_parameter_value().string_value

        # Attempt to load the YAML
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

        # Initialize static broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        # For each tag ID, read x,y,z,roll,pitch,yaw and publish map->tag<ID>_static
        for tag_id, pose in tags.items():
            x     = pose.get('x', 0.0)
            y     = pose.get('y', 0.0)
            z     = pose.get('z', 0.0)
            roll  = pose.get('roll', 0.0)
            pitch = pose.get('pitch', 0.0)
            yaw   = pose.get('yaw', 0.0)

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"  # Parent
            transform.child_frame_id = f"tag{tag_id}_static"  # Child

            # Fill in translation
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            # Convert (roll, pitch, yaw) to quaternion
            q = euler_to_quaternion(roll, pitch, yaw)
            transform.transform.rotation = q

            transforms.append(transform)
            self.get_logger().info(
                f"map->tag{tag_id}_static: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                f"r={roll:.3f}, p={pitch:.3f}, y={yaw:.3f}"
            )

        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("All static transforms (map->tagX_static) published. Node will spin...")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStaticBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
