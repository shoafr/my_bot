#!/usr/bin/env python3
"""
move_to_tag.py (Transform-based version)

Robot flow (example):
1) APPROACH_FIRST:
   - We attempt to see the first AprilTag in the /detections array.
   - Once we lock onto a "first_tag_id", we do a TF lookup from 'camera_link_optical' -> that tag.
   - Move forward until distance < distance_threshold => switch to ROTATE

2) ROTATE:
   - Rotate in place until we detect a *new* tag ID (like "second tag" ID).
   - Switch to APPROACH_SECOND.

3) APPROACH_SECOND:
   - Same as approach first, but for the second tag ID.
   - Move forward until distance < threshold, or do something else.

Key changes from bounding-box version:
- We no longer compute box_area or bounding-box corners.
- We measure distance from camera->tag by reading the transform's translation.
- We define a distance_threshold instead of close_area_ratio.

You can further refine the heading control by computing angles from the transform,
but for simplicity, we'll do a rough center alignment by rotating if we can't see the tag, etc.
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
import math
import time

class MoveToTagNode(Node):
    def __init__(self):
        super().__init__('move_to_tag_node')

        # Subscribe to /detections to find out which tag ID(s) are in view
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF buffer/listener so we can lookup camera->tag transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State machine states
        self.state = 'APPROACH_FIRST'  # 'ROTATE', 'APPROACH_SECOND'
        self.first_tag_id = None

        # Speeds and thresholds
        self.linear_speed = 0.1      # Forward speed (m/s)
        self.rotate_speed = 0.3      # Rotation speed (rad/s)
        self.distance_threshold = 0.3  # Stop distance in meters
        # You can tweak the distance threshold to decide "close enough."

        # For steering control, we'll keep it simple:
        self.angular_speed_factor = 1.0  # factor for small left-right corrections (optional)

        self.get_logger().info("MoveToTagNode started. State=APPROACH_FIRST")

        # Timer or main loop is not strictly needed if we rely on the detection_callback
        # to trigger logic. But we do everything in detection_callback for simplicity.

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        We see which tag IDs are present. If we haven't locked a first_tag_id, pick one.
        Then we do different actions depending on the state.
        """
        # If no detections, handle states that need to spin or stop
        if not msg.detections:
            if self.state == 'APPROACH_FIRST':
                self.stop_robot()
            elif self.state == 'ROTATE':
                self.rotate_in_place()
            elif self.state == 'APPROACH_SECOND':
                self.stop_robot()
            return

        # Let's grab the first detection
        detection = msg.detections[0]
        tag_id = detection.id

        if self.state == 'APPROACH_FIRST':
            # Lock onto first tag ID if none is set
            if self.first_tag_id is None:
                self.first_tag_id = tag_id
                self.get_logger().info(f"First tag ID locked: {self.first_tag_id}")

            # Approach this first tag using TF distance
            self.approach_tag_by_tf(self.first_tag_id)

        elif self.state == 'ROTATE':
            # If we see a *different* tag ID, that becomes the second tag
            if tag_id != self.first_tag_id:
                self.get_logger().info(f"New tag ID {tag_id} detected! Switching to APPROACH_SECOND.")
                self.state = 'APPROACH_SECOND'
                self.stop_robot()
                # Next callback will approach this second tag
            else:
                # Keep rotating until we see a different tag
                self.rotate_in_place()

        elif self.state == 'APPROACH_SECOND':
            # Approach this newly discovered second tag ID
            self.approach_tag_by_tf(tag_id)
            # (Optionally define a second threshold or next state if you want to stop again.)

    def approach_tag_by_tf(self, desired_tag_id):
        """
        Look up the transform from camera_link_optical -> tag<ID>.
        If found, compute distance. If > threshold => move forward, else stop and switch state.
        """
        # Construct the TF frame name for this tag
        # e.g. "tag36h11:5" or "tag5" depending on your apriltag_ros config
        # For simplicity, let's assume it's just "tag{ID}" or "tag5"
        tag_frame = f"tag{desired_tag_id}"

        try:
            # Lookup the transform from camera_link_optical to tagX
            transform_stamped = self.tf_buffer.lookup_transform(
                'camera_link_optical',
                tag_frame,
                rclpy.time.Time()
            )

            # Extract translation
            tx = transform_stamped.transform.translation.x
            ty = transform_stamped.transform.translation.y
            tz = transform_stamped.transform.translation.z

            distance = math.sqrt(tx*tx + ty*ty + tz*tz)

            # Check if we're close enough
            if distance > self.distance_threshold:
                # Move forward
                # Optionally, we might do a small left-right correction
                # if we want to keep the tag centered. For example:
                # Turn left if ty is positive, turn right if ty is negative
                # or you can do a simpler approach => just go straight
                twist = Twist()
                twist.linear.x = self.linear_speed

                # Let's do a mild turning to center the tag:
                # If the tag is to the left (ty>0?), turn left a bit
                # Actually, if we want to keep tag in front, we do
                # an angular z = -some_factor * ty perhaps
                twist.angular.z = -self.angular_speed_factor * ty
                self.cmd_vel_pub.publish(twist)
            else:
                # We are close => switch to next state if we're in APPROACH_FIRST
                if self.state == 'APPROACH_FIRST':
                    self.get_logger().info("Reached first tag. Switching to ROTATE.")
                    self.state = 'ROTATE'
                    self.stop_robot()
                elif self.state == 'APPROACH_SECOND':
                    # Optionally define a next step, or just remain in second approach
                    self.get_logger().info("Reached second tag (approx). Stopping.")
                    self.stop_robot()
                    # If you want a new state, define it here.

        except (LookupException, ExtrapolationException) as ex:
            self.get_logger().warn(f"TF lookup failed for {tag_frame}: {ex}")
            # We might spin in place or stop:
            self.rotate_in_place()

    def rotate_in_place(self):
        """
        Rotate in place
        """
        twist = Twist()
        twist.angular.z = self.rotate_speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """
        Publish zero velocities
        """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
