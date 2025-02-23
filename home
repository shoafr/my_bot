import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)  # Use BCM numbering (GPIO17 instead of Pin 11)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Set GPIO17 as input with pull-down

print("Waiting for trigger...")

try:
    while True:
        if GPIO.input(17) == GPIO.HIGH:
            print("Start Trigger Activated!")
            break  # Exit loop when switch is pressed
        time.sleep(0.1)  # Small delay to reduce CPU usage
finally:
    GPIO.cleanup()  # Clean up GPIO on exit
