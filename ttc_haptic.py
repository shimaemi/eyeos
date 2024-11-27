from tf_luna import TFLuna
import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.out)

pwm = GPIO.PWM(18, 100)
pwm.start(0)

try:
    while True:
        for duty_cycle in range(0, 101, 5):  # Increase duty cycle from 0 to 100
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
        for duty_cycle in range(100, -1, -5):  # Decrease duty cycle from 100 to 0
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()