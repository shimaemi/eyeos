import time
from picamera import PiCamera
import RPLidarA1 as rp

# Initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30

# Initialize lidar
lidar = rp.RPLidar('/dev/ttyUSB0') # Replace with your lidar's port

try:
    while True:
        # Capture image
        camera.capture('image.jpg')

        # Get lidar data
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                print(f"Angle: {angle}, Distance: {distance}")

except KeyboardInterrupt:
    print('Stopping...')
    lidar.stop()
    lidar.disconnect()
    camera.close()
