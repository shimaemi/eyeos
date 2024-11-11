# test to display image keypoints
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from tf_luna import TFLuna

#30 fps
frames = 30
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

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

def keypoints(image):
    # Load the image and convery to grayscale
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect key points and compute descriptors
    keypoints, descriptors = orb.detectAndCompute(img, None)
    for x in keypoints:
        print("({:.2f},{:.2f}) = size {:.2f} angle {:.2f}".format(x.pt[0], x.pt[1], x.size, x.angle))
    
    img_kp = cv2.drawKeypoints(img, keypoints, None,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return img_kp

if __name__ == "__main__":
    try:
        cam_init(frames, IMAGE_WIDTH, IMAGE_HEIGHT)
        # create video capture
        rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

        # Initialize ORB detector
        orb = cv2.ORB_create(20)

        time.sleep(1)
        fps = 0

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # record start time
            start_time = time.time()

            image = frame.array
            img_kp = keypoints(image)
            cv2.imshow("Keypoints", visualize_fps(img_kp, fps))
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

        camera.close()
        cv2.destroyAllWindows()
        print("program interrupted by the user")

    except KeyboardInterrupt:
        camera.close()
        cv2.destroyAllWindows()
        print("program interrupted by the user")