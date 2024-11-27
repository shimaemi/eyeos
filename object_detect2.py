# test to display image bounding boxes
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from tf_luna import TFLuna
from ultralytics import YOLO

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
    results = ncnn_model(frame)

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
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (IMAGE_WIDTH, IMAGE_HEIGHT)})
        picam2.configure(config)
        picam2.start()

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        while(1):
            image = picam2.capture_array()
            cimage = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            img_ob = detect_objects(cimage)
            # cv2.namedWindow("Object Classification", cv2.WINDOW_NORMAL)
            cv2.imshow('Object Classification', visualize_fps(img_ob, fps))
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
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
        picam2.stop()
        cv2.destroyAllWindows()
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print("program interrupted by the user")
