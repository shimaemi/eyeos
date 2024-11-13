# test to display image bounding boxes
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from tf_luna import TFLuna
from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("yolo11n.pt")

#30 fps
frames = 30
IMAGE_WIDTH = model.imgsz[1]
IMAGE_HEIGHT = model.imgsz[0]

#draws a bounding box around each object
def detect_objects(frame):
    # Run YOLO11 tracking on the frame, persisting tracks between frames
    results = model.track(frame, persist=True)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()
    return annotated_frame

if __name__ == "__main__":
    try:
        print("q to quit")

        camera = cam_init(frames, IMAGE_WIDTH, IMAGE_HEIGHT)
        # create video capture
        rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            img_ob = detectObjects(image)
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
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("program interrupted by the user")
