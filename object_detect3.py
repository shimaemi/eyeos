# test to display image bounding boxes
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from tf_luna import TFLuna
from ultralytics import YOLO

# Load a YOLO11n PyTorch model
# creates 'yolo11n_ncnn_model'
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

    # Visualize the results on the frame
    annotated_frame = results[0].plot()
    return annotated_frame

if __name__ == "__main__":
    try:
        print("q to quit")
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (IMAGE_WIDTH, IMAGE_HEIGHT)}))
        camera.start()

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        while(1):
            image = camera.capture_array()

            img_ob = detect_objects(image)

            # clear the stream in preparation for the next frame

            # record end time
            end_time = time.time()
            # calculate FPS
            seconds = end_time - start_time
            fps = 1.0 / seconds
            print("fps: " + str(fps))
            start_time = end_time

    except KeyboardInterrupt:
        camera.close()
    finally:
        camera.close()
        print("program interrupted by the user")
