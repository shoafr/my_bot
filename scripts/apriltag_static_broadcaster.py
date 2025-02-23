#!/usr/bin/env python3
"""
apriltag_static_broadcaster.py

Reads a YAML config of AprilTag positions in the 'map' frame and
publishes static transforms: map -> tag_<id>

Place this in your package (e.g., my_bot/scripts/). Make it executable
and install it via CMakeLists. Then launch it or run it directly to see
the static frames in RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
import math

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

def euler_to_quaternion(yaw):
    """
    Convert yaw (in radians) to a Quaternion (REP-103).
    We'll assume roll = pitch = 0, just a planar rotation.
    """
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class AprilTagStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('apriltag_static_broadcaster')

        # Declare parameter to pass in the YAML path
        self.declare_parameter('tag_positions_file', 'tag_positions.yaml')
        param_file = self.get_parameter('tag_positions_file').get_parameter_value().string_value

        # Load YAML
        try:
            with open(param_file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load {param_file}: {e}")
            return

        if 'tags' not in data:
            self.get_logger().error("No 'tags' dictionary found in YAML.")
            return

        tags = data['tags']

        # Create StaticTransformBroadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        transforms = []
        for tag_id, pose in tags.items():
            # Extract x, y, yaw from the YAML for each tag
            x = pose.get('x', 0.0)
            y = pose.get('y', 0.0)
            yaw = pose.get('yaw', 0.0)

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"               # Parent is map
            transform.child_frame_id = f"tag_{tag_id}"      # Child is e.g. tag_5

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0  # presumably on the ground

            q = euler_to_quaternion(yaw)
            transform.transform.rotation = q

            transforms.append(transform)

            self.get_logger().info(f"Publishing static transform map -> tag_{tag_id} at x={x}, y={y}, yaw={yaw}")

        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("All static transforms published. Node will spin...")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStaticBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
