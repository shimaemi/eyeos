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
    def __init__(self, port, baudrate = 115200):
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
    def set_sample(self, sample = 5):
        # Change the sample rate
        sample_packet = [0x5a,0x06,0x03,sample,00,00] # Sample rate byte array
        self.ser.write(sample_packet) # Send sample rate instruction
        time.sleep(0.1) # Wait or change to take effect
        return 

    # camera.capture('img.jpg')
    # img = cv2.imread('img.jpg')


    def close(self):
        self.ser.close()