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
def activate_haptic_motors(left_intensity, right_intensity):
    """
    Activate haptic motors with specified intensity.
    
    Args:
        left_intensity (int): Intensity for left motor (0-100).
        right_intensity (int): Intensity for right motor (0-100).
    """
    left_pwm.ChangeDutyCycle(left_intensity)  # Set duty cycle for left motor
    right_pwm.ChangeDutyCycle(right_intensity)  # Set duty cycle for right motor
    time.sleep(0.5)  # Keep it on for 0.5 seconds

def deactivate_haptic_motors():
    """
    Deactivate haptic motors.
    """
    left_pwm.ChangeDutyCycle(0)  # Turn off left motor
    right_pwm.ChangeDutyCycle(0)  # Turn off right motor

try:
    while True:
        # Example usage: Activate left motor at 50% intensity and right motor at 75% intensity
        activate_haptic_motors(50, 75)
        time.sleep(1)  # Wait for 1 second before next activation
        # Deactivate motors 
        deactivate_haptic_motors()
        time.sleep(1)  # Wait for 1 second before next activation
        
except KeyboardInterrupt:
    print("Exiting...")
    left_pwm.stop()  # Stop PWM for left haptic motor
    right_pwm.stop() # Stop PWM for right haptic motor
    GPIO.cleanup()  # Clean up GPIO settings