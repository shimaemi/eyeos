import RPI.GPIO as GPIO
import time

# GPIO pins for haptic motors
Left_Haptic_Pin = 17  # GPIO pin for left haptic motor`
Right_Haptic_Pin = 18  # GPIO pin for right haptic motor`

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Pin for haptic left 
GPIO.setup(18, GPIO.OUT)  # Pin for haptic right

# Starting mode off
GPIO.output(Left_Haptic_Pin, GPIO.LOW)  # Set pin to low (off)
GPIO.output(Right_Haptic_Pin, GPIO.LOW)  # Set pin to low (off)

try:
    while True:
        # Activate left haptic motor
        GPIO.output(Left_Haptic_Pin, GPIO.HIGH)
        time.sleep(0.5)  # Keep it on for 0.5 seconds
        GPIO.output(Left_Haptic_Pin, GPIO.LOW)
        time.sleep(0.5)

        # Activate right haptic motor
        GPIO.output(Right_Haptic_Pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(Right_Haptic_Pin, GPIO.LOW)
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.output(Left_Haptic_Pin, GPIO.LOW)  # Set pin to low (off)
    GPIO.output(Right_Haptic_Pin, GPIO.LOW)  # Set pin to low (off)
    GPIO.cleanup()  # Clean up GPIO settings