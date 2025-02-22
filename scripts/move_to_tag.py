#!/usr/bin/env python3
"""
follow_tag.py

Continuously moves the robot toward a detected AprilTag by subscribing to /detections
(from apriltag_ros) and publishing velocity commands to /cmd_vel. This script uses a
simple proportional controller to turn the robot based on how far the tag is from the
image center (in the X-axis).

Place this script into your 'my_bot/scripts' directory (or similar) and ensure it is
installed in the CMakeLists.txt so that it becomes an executable. Then, reference it in
your unified launch file to run continuously.
"""

import rclpy
from rclpy.node import Node

# AprilTagDetectionArray is the message type published by apriltag_ros on /detections
from apriltag_msgs.msg import AprilTagDetectionArray

# We'll publish geometry_msgs/Twist commands to move the robot
from geometry_msgs.msg import Twist

class FollowTagNode(Node):
    """
    Node that continuously moves the robot toward a detected AprilTag.
    - Subscribes to /detections (published by apriltag_ros)
    - Publishes to /cmd_vel to move the robot
    """
    def __init__(self):
        super().__init__('follow_tag_node')

        # Create a subscriber to the /detections topic
        # The queue size is 10 (messages in buffer)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define some internal parameters
        self.image_width = 640    # We'll assume your camera image is 640px wide
        self.half_width = self.image_width / 2.0
        self.linear_speed = 0.1   # Constant forward speed
        self.angular_gain = 0.002 # Gain for turning based on horizontal error

        # Log info for debugging
        self.get_logger().info("FollowTagNode initialized: Subscribed to /detections, publishing to /cmd_vel")

    def detection_callback(self, msg: AprilTagDetectionArray):
        """
        Callback function triggered whenever a new AprilTagDetectionArray message is received.
        We'll process the first detected tag (if any) to move the robot.
        """
        if not msg.detections:
            # If no tags are detected, stop the robot for safety
            self.stop_robot()
            return

        # We'll just track the first detected tag in the array
        detection = msg.detections[0]

        # Check if the detection has corners (the pixel coordinates of the tag)
        # Typically, corners is a list of 4 corner points (x,y).
        if len(detection.corners) < 4:
            self.get_logger().warn("Detection has fewer than 4 corners. Stopping.")
            self.stop_robot()
            return

        # Compute the approximate horizontal center of the tag
        # We'll average the X-coordinates of the corners
        sum_x = 0.0
        for corner in detection.corners:
            sum_x += corner.x
        average_x = sum_x / 4.0

        # Error is how far the tag is from the center of the image
        error_x = average_x - self.half_width

        # Prepare a Twist message to move the robot
        twist = Twist()

        # We'll move forward at a constant speed
        twist.linear.x = self.linear_speed

        # We'll turn based on how far from center the tag is
        # Negative error => tag is to the left => turn left => angular.z is positive
        # Positive error => tag is to the right => turn right => angular.z is negative
        twist.angular.z = -error_x * self.angular_gain

        # Finally, publish the velocity command
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(
            f"Tag found. average_x={average_x:.2f}, error_x={error_x:.2f}, angular_z={twist.angular.z:.3f}"
        )

    def stop_robot(self):
        """
        Publishes a zero velocity message to stop the robot.
        """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    """
    Entry point: Initialize rclpy, create the FollowTagNode, and spin until shutdown.
    """
    rclpy.init(args=args)
    node = FollowTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
