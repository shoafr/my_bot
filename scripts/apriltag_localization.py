#!/usr/bin/env python3
"""
apriltag_localization.py

Subscribes to /detections, inverts the transform from base_link->tag_<id> to get map->base_link,
assuming map->tag_<id> is published statically. We then broadcast map->base_link continuously so
you can see the robot's pose in RViz.

This script does not command any motion; it only updates your TF tree.
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException
import math
import time

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # Create a TF buffer & listener to look up 'map->tag_<id>' from the static broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a broadcaster to publish 'map->base_link'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the AprilTag detections topic
        self.sub_detections = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )

        # For controlling update frequency
        self.last_update_time = time.time()
        self.update_rate = 5.0  # Hz (maximum number of times we'll update TF per second)

        self.get_logger().info("AprilTagLocalization initialized, waiting for /detections...")

    def detections_callback(self, msg):
        # Limit update rate
        now = time.time()
        if (now - self.last_update_time) < (1.0 / self.update_rate):
            return
        self.last_update_time = now

        if not msg.detections:
            # No tags => no position update
            return

        # We'll just take the first detection for now (or you could pick the nearest, etc.)
        detection = msg.detections[0]
        if len(detection.corners) < 4:
            return

        # detection.id is typically an integer
        tag_id = detection.id
        tag_frame = f"tag_{tag_id}"

        # apriltag_ros might provide a full pose in detection.pose.pose if you use 'publish_tf' param
        # but let's illustrate the 'invert transforms' approach, which is more general.
        try:
            # We want: map->base_link
            # We already have: map->tag_id (static) and base_link->tag_id from detection
            # But detection only gives corners, not a direct TF. So let's see if the apriltag_ros
            # provides a detection.pose or we must rely on corners.

            # If apriltag_ros publishes detection.pose, let's assume detection.pose.pose:
            # (In older versions, detection might have a 'pose' or 'centre'. YMMV.)
            # If not, you might need a separate step to convert corners to a TF.

            if not detection.pose.pose:
                # If your version doesn't provide a direct pose, you must do geometry from corners.
                # Skipping for brevity. We'll assume detection.pose.pose is valid for demonstration.
                return

            # We'll look up 'map->tag_<id>' from the TF tree
            map_to_tag = self.tf_buffer.lookup_transform(
                'map',
                tag_frame,  # e.g. 'tag_5'
                rclpy.time.Time()
            )
            # This gives us the known static transform from map->tag

            # Next, we have detection.pose.pose => base_link->tag in some references
            # Actually, apriltag_ros might define the pose as tag->camera or camera->tag
            # Depending on the param 'z_up' or 'camera_frame'. We want base_link->tag explicitly,
            # so you may need to transform camera->tag to base_link->tag via the camera->base_link offset.

            # For simplicity, let's say detection.pose.pose is camera->tag. Then do:
            # camera->tag * base_link->camera = base_link->tag. We'll look up base_link->camera from TF
            # Or if it's already base_link->tag, we can skip.

            # Let's assume detection.pose.pose is base_link->tag for demonstration:
            base_link_to_tag = detection.pose.pose

            # Now invert base_link->tag => tag->base_link
            tag_to_base_link = invert_pose(base_link_to_tag)

            # Then compose map->tag with tag->base_link to get map->base_link
            map_to_base_link = multiply_transforms(map_to_tag.transform, tag_to_base_link)

            # Publish map->base_link
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform = map_to_base_link
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().debug(f"Updated map->base_link from tag {tag_id}")

        except LookupException as ex:
            self.get_logger().warn(f"Could not find map->tag_{tag_id} transform: {ex}")
            return

def invert_pose(pose):
    """
    Invert a 2D or 3D pose in geometry_msgs/Pose form. 
    For demonstration, let's do a naive approach:
    - we treat (x, y, z) + quaternion as a typical 3D transform
    - if you have an advanced approach, use tf_transformations or similar
    """
    # We'll rely on tf2 if possible, but let's do a simplistic approach
    import tf_transformations as tft
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    # Convert to matrix
    mat = tft.quaternion_matrix(rot)
    mat[0, 3] = trans[0]
    mat[1, 3] = trans[1]
    mat[2, 3] = trans[2]

    # Invert
    inv = tft.inverse_matrix(mat)
    # Extract translation
    trans_inv = (inv[0, 3], inv[1, 3], inv[2, 3])
    # Extract quaternion
    rot_inv = tft.quaternion_from_matrix(inv)

    # Construct a new Pose
    from geometry_msgs.msg import Transform
    result = Transform()
    result.translation.x = trans_inv[0]
    result.translation.y = trans_inv[1]
    result.translation.z = trans_inv[2]
    result.rotation.x = rot_inv[0]
    result.rotation.y = rot_inv[1]
    result.rotation.z = rot_inv[2]
    result.rotation.w = rot_inv[3]
    return result

def multiply_transforms(t1, t2):
    """
    Multiply geometry_msgs/Transform t1 * t2 => t_out
    Where t1, t2 are 'map->tag' and 'tag->base_link' => map->base_link
    """
    import tf_transformations as tft

    # Convert t1 to matrix
    trans1 = (t1.translation.x, t1.translation.y, t1.translation.z)
    rot1   = (t1.rotation.x, t1.rotation.y, t1.rotation.z, t1.rotation.w)
    mat1 = tft.quaternion_matrix(rot1)
    mat1[0,3] = trans1[0]
    mat1[1,3] = trans1[1]
    mat1[2,3] = trans1[2]

    # Convert t2 to matrix
    trans2 = (t2.translation.x, t2.translation.y, t2.translation.z)
    rot2   = (t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w)
    mat2 = tft.quaternion_matrix(rot2)
    mat2[0,3] = trans2[0]
    mat2[1,3] = trans2[1]
    mat2[2,3] = trans2[2]

    mat_out = tft.concatenate_matrices(mat1, mat2)
    # Convert back to geometry_msgs/Transform
    trans_out = TransformStamped()
    # We'll just fill .transform in a Transform type
    out = Transform()
    out_tf = tft.decompose_matrix(mat_out)
    # out_tf = (scale, shear, rpy, translation)
    # We'll just get translation & quaternion
    # parse translation
    out.translation.x = out_tf[3][0]
    out.translation.y = out_tf[3][1]
    out.translation.z = out_tf[3][2]
    # Convert roll-pitch-yaw back to quaternion or just from mat_out
    quat = tft.quaternion_from_matrix(mat_out)
    out.rotation.x = quat[0]
    out.rotation.y = quat[1]
    out.rotation.z = quat[2]
    out.rotation.w = quat[3]
    return out

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
