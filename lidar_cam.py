# test to display image bounding boxes
import serial # uart
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from tf_luna import TFLuna
from ultralytics import YOLO
import lgpio

ser = serial.Serial('/dev/serial0', 115200)
# we define a new function that will get the data from LiDAR and publish it
sample = 10 # set sample rate 5 / sec
t = 1 / sample # period

# Load a YOLO11n PyTorch model
# Load the exported NCNN model
ncnn_model = YOLO("yolo11n_ncnn_model")

#30 fps
frames = 30
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# for testing
def visualize_fps(image, fps: int):
    if len(np.shape(image)) < 3:
        text_color = (255, 255, 255)  # white
    else:
        text_color = (0, 255, 0)  # green
    row_size = 20  # pixels
    left_margin = 24  # pixels

    font_size = 1
    font_thickness = 1

    # Draw the FPS counter
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)
    return image

#draws a bounding box around each object
def detect_objects(frame):
    # Run YOLO11 tracking on the frame, persisting tracks between frames
    results = ncnn_model.track(frame, persist=True)

    # Draw bounding boxes and labels on the frame
    for result in results[0].boxes:  # Access the bounding boxes directly
        x1, y1, x2, y2 = result.xyxy[0]
        confidence = result.conf[0]
        class_id = result.cls[0]
        label = model.names[int(class_id)]
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {confidence:.2f}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return frame

if __name__ == "__main__":
    try:
        print("q to quit")

        # Initialize the TFLuna sensor
        sensor = TFLuna(port='/dev/ttyAMA0', baudrate=115200, pwm_pin=18)
        # Set the sample rate
        sensor.set_sample(100)  # Set to 100 Hz or your desired sample rate
        period = self.get_period()
        prev = 0
        
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (IMAGE_WIDTH, IMAGE_HEIGHT)}))
        camera.start()

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        while(1):
            counter = self.ser.in_waiting  # count the number of bytes of the serial port
            if counter > 8:
                bytes_serial = self.ser.read(9)
                self.ser.reset_input_buffer()
        
                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # python3
                    curr = bytes_serial[2] + bytes_serial[3] * 256  # centimeters
                    if curr != prev:
                        ttc = curr * period / (prev - curr)  # .01 = time between two measurements in seconds, 1 / framerate (100hz default)
                        ttc = round(ttc, 5)
                        print("TTC:" + str(ttc) + " sec")
                        self.set_vibration_intensity(ttc)  # Adjust vibration intensity based on TTC
                        prev = curr
                    self.ser.reset_input_buffer()

            image = camera.capture_array()

            img_ob = detect_objects(image)
            # cv2.namedWindow("Object Classification", cv2.WINDOW_NORMAL)
            cv2.imshow('Object Classification', visualize_fps(img_ob, fps))
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

            # record end time
            end_time = time.time()
            # calculate FPS
            seconds = end_time - start_time
            fps = 1.0 / seconds
            start_time = end_time

    except KeyboardInterrupt:
        camera.close()
        cv2.destroyAllWindows()
        sensor.close()
    finally:
        camera.close()
        cv2.destroyAllWindows()
        sensor.close()
        print("program interrupted by the user")
