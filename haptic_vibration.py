import RPi.GPIO as GPIO
import time
import random  # Import the random module

# GPIO setup for motors
MOTOR_LEFT = 17     # GPIO17 for Left Motor
MOTOR_RIGHT = 27    # GPIO27 for Right Motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT, GPIO.OUT)

# Start with motors off
GPIO.output(MOTOR_LEFT, GPIO.LOW)
GPIO.output(MOTOR_RIGHT, GPIO.LOW)

try:
    while True:
        # Simulate direction detection (replace this with actual sensor input)
        direction = random.choice(["left", "right"])  # Randomly choose left or right

        if direction == "left":
            print("🚨 Leftward motion detected! Vibrating Left Motor...")
            GPIO.output(MOTOR_LEFT, GPIO.HIGH)  # Turn ON Left Motor
            time.sleep(1)  # Vibrate for 1 second
            GPIO.output(MOTOR_LEFT, GPIO.LOW)  # Turn OFF Left Motor

        elif direction == "right":
            print("🚨 Rightward motion detected! Vibrating Right Motor...")
            GPIO.output(MOTOR_RIGHT, GPIO.HIGH)  # Turn ON Right Motor
            time.sleep(1)  # Vibrate for 1 second
            GPIO.output(MOTOR_RIGHT, GPIO.LOW)  # Turn OFF Right Motor

        time.sleep(2)  # Wait for a while before detecting next direction

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()  # Clean up GPIO pins when exiting
