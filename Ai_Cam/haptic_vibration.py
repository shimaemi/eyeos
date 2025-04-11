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


# Function to activate haptic motors
def activate_haptic_left(left_intensity):
    left_pwm.ChangeDutyCycle(left_intensity)  # Set duty cycle for left motor
    time.sleep(0.5)  # Keep it on for 0.5 seconds

def activate_haptic_right(right_intensity):
    right_pwm.ChangeDutyCycle(right_intensity)  # Set duty cycle for right motor
    time.sleep(0.5)  # Keep it on for 0.5 seconds

def deactivate_haptic_left():
    left_pwm.ChangeDutyCycle(0)  # Turn off left motor

def deactivate_haptic_right():
    right_pwm.ChangeDutyCycle(0)  # Turn off right motor

try:
    while True:
        # Example usage: Activate left motor at 50% intensity and right motor at 75% intensity
        activate_haptic_left(100)
        time.sleep(1)
        deactivate_haptic_left()
        time.sleep(1)
        activate_haptic_right(100)
        time.sleep(1)
        deactivate_haptic_right()
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Exiting...")
    left_pwm.stop()  # Stop PWM for left haptic motor
    right_pwm.stop() # Stop PWM for right haptic motor
    GPIO.cleanup()  # Clean up GPIO settings