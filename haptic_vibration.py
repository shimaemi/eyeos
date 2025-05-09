import RPi.GPIO as GPIO
import time
import threading

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
        
        # Cooldown Tracking
        self.last_activation = 0  # Single timestamp for both motors
        self.cooldown = 1.5  # seconds between activations

    def activate_left(self, intensity=100, duration=0.3):
        """Activate left motor synchronized with audio"""
        current_time = time.time()
        if current_time - self.last_activation > self.cooldown:
            try:
                self.last_activation = current_time
                self.left_pwm.ChangeDutyCycle(intensity)
                time.sleep(duration)
                self.left_pwm.ChangeDutyCycle(0)
                return True  # Success
            except Exception as e:
                print(f"Haptic error (left): {e}")
        return False  # Skipped due to cooldown

    def activate_right(self, intensity=100, duration=0.3):
        """Activate right motor synchronized with audio"""
        current_time = time.time()
        if current_time - self.last_activation > self.cooldown:
            try:
                self.last_activation = current_time
                self.right_pwm.ChangeDutyCycle(intensity)
                time.sleep(duration)
                self.right_pwm.ChangeDutyCycle(0)
                return True  # Success
            except Exception as e:
                print(f"Haptic error (right): {e}")
        return False  # Skipped due to cooldown

    def activate_both(self, intensity=100, duration=0.3):
        """Activate both motors synchronized with audio"""
        current_time = time.time()
        if current_time - self.last_activation > self.cooldown:
            try:
                self.last_activation = current_time
                self.left_pwm.ChangeDutyCycle(intensity)
                self.right_pwm.ChangeDutyCycle(intensity)
                time.sleep(duration)
                self.left_pwm.ChangeDutyCycle(0)
                self.right_pwm.ChangeDutyCycle(0)
                return True  # Success
            except Exception as e:
                print(f"Haptic error (both): {e}")
        return False  # Skipped due to cooldown

    def deactivate_all(self):
        """Immediately turn off both motors."""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

    def set_cooldown(self, seconds):
        """Set the cooldown period for the motors."""
        self.cooldown = max(0.1, seconds)  # minimum 0.1 second cooldown

    def cleanup(self):
        """Clean up GPIO resources. Call this before exiting."""
        self.deactivate_all()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()