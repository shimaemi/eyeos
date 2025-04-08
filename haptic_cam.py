import time
import RPi.GPIO as GPIO
import cv2
import numpy as np

# Define GPIO pins for the motors
MOTOR_RIGHT = 17  # Right motor on GPIO pin 17
MOTOR_LEFT= 18   # Left motor on GPIO pin 18

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_RIGHT, GPIO.OUT)
GPIO.setup(MOTOR_LEFT, GPIO.OUT)

# Camera setup
cap = cv2.VideoCapture(0)  # Open the camera

# Initialize the previous frame for motion detection
prev_frame = None

class VibrationMotor:
    def __init__(self, motor_pin):
        self.motor_pin = motor_pin

    def activate(self):
        GPIO.output(self.motor_pin, GPIO.HIGH)  # Activate the motor (turn on the pin)

    def deactivate(self):
        GPIO.output(self.motor_pin, GPIO.LOW)  # Deactivate the motor (turn off the pin)

class MotionDetection:
    def __init__(self):
        self.prev_frame = None

    def detect_motion_direction(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Initialize the first frame for motion comparison
        if self.prev_frame is None:
            self.prev_frame = gray
            return None

        # Compute the absolute difference between the current frame and the previous frame
        frame_delta = cv2.absdiff(self.prev_frame, gray)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

        # Dilate the thresholded image to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)

        # Find contours of the moving areas
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Reset previous frame
        self.prev_frame = gray

        # If no contours are detected, return None (no motion)
        if len(contours) == 0:
            return None

        # If contours are detected, analyze direction (left or right)
        direction = self.analyze_motion_direction(contours)
        return direction

    def analyze_motion_direction(self, contours):
        # Here, we assume the largest contour is the one representing the most significant motion
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding box of the largest contour
        (x, y, w, h) = cv2.boundingRect(largest_contour)

        # Check the horizontal movement of the bounding box
        if w > h:  # Consider it horizontal motion
            if x < 160:  # Motion is detected on the left side of the screen
                return 'left'
            elif x + w > 480:  # Motion is detected on the right side of the screen
                return 'right'
        return None

class VibrationMotorController:
    def __init__(self):
        self.motor_left = VibrationMotor(MOTOR_LEFT)  # Motor for left direction
        self.motor_right = VibrationMotor(MOTOR_RIGHT)  # Motor for right direction
        self.motion_detector = MotionDetection()
    
    def detect_and_activate(self, direction):
        if direction == 'left':
            self.motor_left.activate()
            self.motor_right.deactivate()
        elif direction == 'right':
            self.motor_right.activate()
            self.motor_left.deactivate()
        else:
            self.motor_left.deactivate()
            self.motor_right.deactivate()

if __name__ == "__main__":
    controller = VibrationMotorController()

    try:
        while True:
            # Capture frame from the camera
            ret, frame = cap.read()

            # If frame is not captured, continue the loop
            if not ret:
                continue

            # Detect the direction of motion
            direction = controller.motion_detector.detect_motion_direction(frame)

            # If motion detected, activate appropriate motor
            controller.detect_and_activate(direction)

            # Display the frame (for debugging purposes)
            cv2.imshow("Frame", frame)

            # Wait for key press to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)  # Delay for smoother processing

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        # Cleanup GPIO and release the camera
        cap.release()
        GPIO.cleanup()
        cv2.destroyAllWindows()
