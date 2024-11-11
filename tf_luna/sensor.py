# sensor.py
import serial  # type: ignore
import time
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

class TFLuna:
    # port = serial port TF-Luna is connected to (~AMA0)
    # baudrate = communication speed default is set
    def lid_init(self, port, baudrate = 115200):
        self.ser = serial.Serial(port, baudrate, timeout = 0)
        self.prev = 0
        self.sample_rate = None
        self.period = None

    # Initialize camera
    def cam_init(self, frames, IMAGE_WIDTH, IMAGE_HEIGHT):
        camera = PiCamera()
        camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
        camera.framerate = frames

    # for lidar
    def set_lid_samp(self, sample):
        # Change the sample rate
        sample_packet = [0x5a,0x06,0x03,sample,00,00] # Sample rate byte array
        self.ser.write(sample_packet) # Send sample rate instruction
        time.sleep(0.1) # Wait or change to take effect
        return 

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

    def close(self):
        self.ser.close()

    # camera.capture('img.jpg')
    # img = cv2.imread('img.jpg')