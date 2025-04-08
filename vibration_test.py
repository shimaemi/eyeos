import RPi.GPIO as GPIO
import time

vibration_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(vibration_pin, GPIO.OUT)

def vibrate():
    GPIO.output(vibration_pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(vibration_pin, GPIO.LOW)

vibrate()

GPIO.cleanup()
