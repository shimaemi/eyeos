#First Testing Code using time for l/r direction:

import RPi.GPIO as GPIO
import time
import random

# GPIO setup
MOTOR_LEFT = 17  # GPIO17 for Left Motor
MOTOR_RIGHT = 27  # GPIO27 for Right Motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT, GPIO.OUT)

try:
    while True:
        direction = random.choice(["left", "right"])  # Random left/right

        if direction == "left":
            print("Leftward motion detected! Vibrating Left Motor...")
            GPIO.output(MOTOR_LEFT, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(MOTOR_LEFT, GPIO.LOW)

        elif direction == "right":
            print("Rightward motion detected! Vibrating Right Motor...")
            GPIO.output(MOTOR_RIGHT, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(MOTOR_RIGHT, GPIO.LOW)

      #caused too much vibration  time.sleep(random.randint(3, 7))  # Random delay before next detection

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()

