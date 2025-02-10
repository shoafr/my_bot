import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class StartTriggerNode(Node):
    def __init__(self):
        super().__init__('start_trigger')
        
        # Define GPIO pin for LED or switch
        self.trigger_pin = 17  # Update with actual GPIO pin number
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Publisher to notify when the robot should start
        self.trigger_publisher = self.create_publisher(Bool, 'start_signal', 10)
        
        # Timer to check the trigger status periodically
        self.timer = self.create_timer(0.1, self.check_trigger)

    def check_trigger(self):
        if GPIO.input(self.trigger_pin) == GPIO.HIGH:
            msg = Bool()
            msg.data = True
            self.trigger_publisher.publish(msg)
            self.get_logger().info("Start signal received! Publishing to /start_signal")


def main(args=None):
    rclpy.init(args=args)
    node = StartTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
