# test to display image bounding boxes
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from tf_luna import TFLuna

# Load YOLO model
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
classes = []
# Load class names from coco file
with open("coco.names", "r") as f:
    classes = f.read().strip().split("\n")

#30 fps
frames = 30
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

#draws a bounding box around each object
def detect_objects(frame):
    # Preprocess the frame for YOLO
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(net.getUnconnectedOutLayersNames())

    # Process YOLO output
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                # Extract detection details
                center_x = int(detection[0] * frame.shape[1])
                center_y = int(detection[1] * frame.shape[0])
                width = int(detection[2] * frame.shape[1])
                height = int(detection[3] * frame.shape[0])

                x = int(center_x - width / 2)
                y = int(center_y - height / 2)

                # Draw a box and label on the frame
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
                label = f"{classes[class_id]}: {confidence:.2f}"
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return frame

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





# download config files from
# https://github.com/yashrajmani/OpenCV_Yolo3_Object_Detection-from-Video/tree/main