import RPi.GPIO as GPIO
import time

# GPIO pins for haptic motors
Left_Haptic_Pin = 17  # GPIO pin for left haptic motor`
Right_Haptic_Pin = 18  # GPIO pin for right haptic motor`

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Pin for haptic left 
GPIO.setup(18, GPIO.OUT)  # Pin for haptic right

# PWM frequency and duty cycle
left_pwm = GPIO.PWM(Left_Haptic_Pin, 1000)  # 1kHz frequency
right_pwm = GPIO.PWM(Right_Haptic_Pin, 1000)  # 1kHz frequency

# Starting mode off
left_pwm.start(0)  # Start PWM with 0% duty cycle (off)
right_pwm.start(0)  # Start PWM with 0% duty cycle (off)

try:
    while True:
        # Activate left haptic motor
        left_pwm.ChangeDutyCycle(100)  # Set duty cycle to 100%
        time.sleep(0.5)  # Keep it on for 0.5 seconds
        left_pwm.ChangeDutyCycle(0)
        time.sleep(0.5)

        # Activate right haptic motor
        right_pwm.ChangeDutyCycle(75)
        time.sleep(0.5)
        right_pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("Exiting...")
    left_pwm.stop()  # Stop PWM for left haptic motor
    right_pwm.stop() # Stop PWM for right haptic motor
    GPIO.cleanup()  # Clean up GPIO settings