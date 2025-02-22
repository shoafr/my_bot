import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class MoveToAprilTag(Node):
    def __init__(self):
        super().__init__('move_to_apriltag')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_id = 5  # AprilTag ID to track
        self.tag_detected = False

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.id == self.target_id:
                self.tag_detected = True
                self.move_toward_tag(detection.centre)
                return
        self.tag_detected = False
        self.stop_robot()

    def move_toward_tag(self, centre):
        twist = Twist()
        linear_speed = 0.1
        angular_speed = 0.1
        
        # Adjust movement based on tag's position
        image_centre_x = 320  # Assuming 640px width
        error_x = centre.x - image_centre_x
        twist.linear.x = linear_speed
        twist.angular.z = -error_x * 0.002  # Proportional turning
        self.publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToAprilTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
