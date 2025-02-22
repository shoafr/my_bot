#!/usr/bin/env python3
"""
move_to_tag.py

Robot flow:
1) APPROACH_FIRST:
   - Move forward & steer towards the first AprilTag detection
   - Once bounding-box area > threshold => transition to ROTATE

2) ROTATE:
   - Rotate in place indefinitely
   - Once a NEW tag (different ID) is detected => transition to APPROACH_SECOND

3) APPROACH_SECOND:
   - Approach the newly-detected second tag (like the first stage)
   - Optionally, you could define another bounding-box threshold to stop again.

Feel free to tweak bounding-box thresholds, speeds, and logic to match your robot.
"""

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import time

class MoveToTagNode(Node):
    def __init__(self):
        super().__init__('move_to_tag_node')

        # Subscribe to /detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State machine states
        self.state = 'APPROACH_FIRST'   # or 'ROTATE', 'APPROACH_SECOND'

        # Storage for first tag ID
        self.first_tag_id = None

        # Basic camera info (tweak if camera is bigger/smaller)
        self.image_width = 640
        self.image_height = 480
        self.half_width = self.image_width / 2.0

        # Speed and gains
        self.linear_speed = 0.1
        self.angular_gain = 0.002

        # Area threshold: if bounding-box area is greater than X% of total image => close enough
        self.close_area_ratio = 0.20  # 20% of image area => "close"

        # Rotation speed
        self.rotate_speed = 0.3  # rad/s (positive = CCW)

        # Logging
        self.get_logger().info("MoveToTagNode started. State=APPROACH_FIRST")

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        Main logic callback for AprilTag detections.
        """
        if not msg.detections:
            # No detections => if we're rotating or approaching, we might just keep doing it
            if self.state == 'APPROACH_FIRST':
                # We'll keep going forward, but no tag => better to stop
                self.stop_robot()
            elif self.state == 'ROTATE':
                # Keep rotating until new tag is found
                self.rotate_in_place()
            elif self.state == 'APPROACH_SECOND':
                self.stop_robot()
            return

        # If we do see a detection, let's take the first in the array
        detection = msg.detections[0]

        # Calculate bounding-box area
        if len(detection.corners) < 4:
            self.stop_robot()
            return

        box_width = abs(detection.corners[0].x - detection.corners[2].x)
        box_height = abs(detection.corners[0].y - detection.corners[2].y)
        box_area = box_width * box_height
        image_area = self.image_width * self.image_height

        # # ID logic: detection.id is typically an array (like detection.id[0]) in apriltag_msgs
        # if len(detection.id) == 0:
        #     self.get_logger().warn("Tag detection has no ID array. Stopping.")
        #     self.stop_robot()
        #     return
        tag_id = detection.id

        # State Machine
        if self.state == 'APPROACH_FIRST':
            # If we haven't stored the first tag ID, do so now
            if self.first_tag_id is None:
                self.first_tag_id = tag_id
                self.get_logger().info(f"First tag ID locked: {self.first_tag_id}")

            # Approach the first tag
            self.approach_tag(detection)

            # Check if we are close enough
            if box_area > self.close_area_ratio * image_area:
                # Switch to ROTATE state
                self.get_logger().info("Close to first tag. Switching to ROTATE state.")
                self.state = 'ROTATE'
                # Stop moving forward
                self.stop_robot()

        elif self.state == 'ROTATE':
            # We are rotating in place. Keep rotating until we see a *different* tag
            if tag_id != self.first_tag_id:
                # Found a new tag! Approach it
                self.get_logger().info(f"New tag ID {tag_id} detected. Switching to APPROACH_SECOND.")
                self.state = 'APPROACH_SECOND'
                self.stop_robot()
                # Approach second tag on next callback
            else:
                # Same old tag => keep rotating
                self.rotate_in_place()

        elif self.state == 'APPROACH_SECOND':
            # Approach the newly detected second tag
            self.approach_tag(detection)
            # Optionally, you could also add a second bounding-box threshold here
            # if box_area > something => do something else
            pass

    def approach_tag(self, detection):
        """
        Move forward, steering based on the horizontal error from image center.
        """
        # Compute approximate horizontal center of bounding box
        sum_x = 0.0
        for corner in detection.corners:
            sum_x += corner.x
        average_x = sum_x / 4.0

        error_x = average_x - self.half_width

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = -error_x * self.angular_gain
        self.cmd_vel_pub.publish(twist)

    def rotate_in_place(self):
        """
        Rotate in place CCW at rotate_speed.
        """
        twist = Twist()
        twist.angular.z = self.rotate_speed  # CCW
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """
        Publish zero velocities to stop.
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
