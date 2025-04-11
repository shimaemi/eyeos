import RPi.GPIO as GPIO
import time

class HapticController:
    def __init__(self, left_pin=17, right_pin=18, pwm_frequency=1000):
        """Initialize the haptic controller with GPIO pins and PWM settings."""
        self.Left_Haptic_Pin = left_pin
        self.Right_Haptic_Pin = right_pin
        self.PWM_Frequency = pwm_frequency
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Left_Haptic_Pin, GPIO.OUT)
        GPIO.setup(self.Right_Haptic_Pin, GPIO.OUT)
        
        # Initialize PWM
        self.left_pwm = GPIO.PWM(self.Left_Haptic_Pin, self.PWM_Frequency)
        self.right_pwm = GPIO.PWM(self.Right_Haptic_Pin, self.PWM_Frequency)
        
        # Start with motors off
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def activate_left(self, intensity=100, duration=0.5):
        """Activate the left motor with a given intensity (0-100) and duration (seconds)."""
        self.left_pwm.ChangeDutyCycle(intensity)
        time.sleep(duration)
        self.left_pwm.ChangeDutyCycle(0)  # Turn off after duration
    
    def activate_right(self, intensity=100, duration=0.5):
        """Activate the right motor with a given intensity (0-100) and duration (seconds)."""
        self.right_pwm.ChangeDutyCycle(intensity)
        time.sleep(duration)
        self.right_pwm.ChangeDutyCycle(0)  # Turn off after duration
    
    def deactivate_all(self):
        """Turn off both motors."""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        """Clean up GPIO resources. Call this before exiting."""
        self.deactivate_all()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()