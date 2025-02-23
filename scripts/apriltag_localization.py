#!/usr/bin/env python3
"""
apriltag_localization.py

1) Subscribes to /detections (AprilTagDetectionArray) from apriltag_ros
2) For each detected tag ID, looks up:
   - map -> tag<ID>_static  (the static frame you renamed in your static broadcaster)
   - camera_link_optical -> tag<ID> (published by apriltag_ros)
3) We invert camera_link_optical->tag<ID> => tag<ID>->camera_link_optical
4) Multiply map->tag<ID>_static * tag<ID>->camera_link_optical => map->camera_link_optical
5) Broadcast that dynamic transform for real-time visualization in RViz

We do NOT move the robot. This only publishes the robot's camera frame in 'map'.

IMPORTANT:
- The static broadcaster now uses 'tag{ID}_static' as the child frame.
- apriltag_ros uses 'tag{ID}' as child frame. So effectively 'tag5_static' != 'tag5' in TF. 
- We unify them here in code.

Steps to use:
1) Launch your static transforms node, which publishes map->tag5_static
2) Launch apriltag_ros, which publishes camera_link_optical->tag5
3) Run this node. 
4) In RViz, set 'map' as the Fixed Frame, see 'camera_link_optical' update in real-time.
"""

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Transform
import tf_transformations as tft

from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException
import time

def invert_transform(transform_in: Transform) -> Transform:
    """
    Inverts a geometry_msgs/Transform using tf_transformations.
    """
    trans = (transform_in.translation.x,
             transform_in.translation.y,
             transform_in.translation.z)
    rot   = (transform_in.rotation.x,
             transform_in.rotation.y,
             transform_in.rotation.z,
             transform_in.rotation.w)

    mat = tft.quaternion_matrix(rot)
    mat[0, 3] = trans[0]
    mat[1, 3] = trans[1]
    mat[2, 3] = trans[2]

    inv = tft.inverse_matrix(mat)

    out = Transform()
    out.translation.x = inv[0, 3]
    out.translation.y = inv[1, 3]
    out.translation.z = inv[2, 3]
    q = tft.quaternion_from_matrix(inv)
    out.rotation.x = q[0]
    out.rotation.y = q[1]
    out.rotation.z = q[2]
    out.rotation.w = q[3]
    return out

def multiply_transforms(t1: Transform, t2: Transform) -> Transform:
    """
    Composes geometry_msgs/Transform t1 * t2 => t_out.
    E.g. map->tag5_static * tag5->camera_link_optical = map->camera_link_optical
    """
    trans1 = (t1.translation.x, t1.translation.y, t1.translation.z)
    rot1   = (t1.rotation.x, t1.rotation.y, t1.rotation.z, t1.rotation.w)
    mat1 = tft.quaternion_matrix(rot1)
    mat1[0,3] = trans1[0]
    mat1[1,3] = trans1[1]
    mat1[2,3] = trans1[2]

    trans2 = (t2.translation.x, t2.translation.y, t2.translation.z)
    rot2   = (t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w)
    mat2 = tft.quaternion_matrix(rot2)
    mat2[0,3] = trans2[0]
    mat2[1,3] = trans2[1]
    mat2[2,3] = trans2[2]

    mat_out = tft.concatenate_matrices(mat1, mat2)

    out = Transform()
    out.translation.x = mat_out[0,3]
    out.translation.y = mat_out[1,3]
    out.translation.z = mat_out[2,3]
    q = tft.quaternion_from_matrix(mat_out)
    out.rotation.x = q[0]
    out.rotation.y = q[1]
    out.rotation.z = q[2]
    out.rotation.w = q[3]
    return out

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # We'll store a TF buffer & listener to look up transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # We'll broadcast 'map -> camera_link_optical'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the AprilTag detections
        self.sub_detections = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.on_detections,
            10
        )

        # Rate limit the updates to 5 Hz
        self.last_update_time = time.time()
        self.update_rate = 5.0

        self.get_logger().info("AprilTagLocalization started. Listening to /detections...")

    def on_detections(self, msg: AprilTagDetectionArray):
        now = time.time()
        if (now - self.last_update_time) < (1.0 / self.update_rate):
            return
        self.last_update_time = now

        if not msg.detections:
            return  # no tags => no update

        # Just use the first detection
        detection = msg.detections[0]
        tag_id = detection.id  # e.g. 5

        # The static transform is: map->tag5_static
        # The apriltag_ros transform is: camera_link_optical->tag5
        static_tag_frame = f"tag{tag_id}_static"
        dynamic_tag_frame = f"tag{tag_id}"

        try:
            # 1) map->tag5_static
            map_to_tag_static = self.tf_buffer.lookup_transform(
                'map',
                static_tag_frame,
                rclpy.time.Time()
            )

            # 2) camera_link_optical->tag5
            cam_to_tag = self.tf_buffer.lookup_transform(
                'camera_link_optical',
                dynamic_tag_frame,
                rclpy.time.Time()
            )

            # Invert camera_link_optical->tag5 => tag5->camera_link_optical
            tag_to_cam = invert_transform(cam_to_tag.transform)

            # Compose map->tag5_static * tag5->camera_link_optical => map->camera_link_optical
            map_to_cam = multiply_transforms(map_to_tag_static.transform, tag_to_cam)

            # Broadcast map->camera_link_optical
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = self.get_clock().now().to_msg()
            tf_stamped.header.frame_id = 'map'
            tf_stamped.child_frame_id = 'camera_link_optical'
            tf_stamped.transform = map_to_cam

            self.tf_broadcaster.sendTransform(tf_stamped)

            self.get_logger().debug(f"Updated map->camera_link_optical from tag{tag_id}")

        except LookupException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
