# test to display image keypoints
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from tf_luna import TFLuna

# 30 fps by default
# frames = 30
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

def keypoints(image):
    
    # Detect key points and compute descriptors
    keypoints, descriptors = orb.detectAndCompute(image, None)
    # for x in keypoints:
        # print("({:.2f},{:.2f}) = size {:.2f} angle {:.2f}".format(x.pt[0], x.pt[1], x.size, x.angle))
    
    img_kp = cv2.drawKeypoints(image, keypoints, None,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return img_kp

if __name__ == "__main__":
    try:
        print("q to quit")
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (IMAGE_WIDTH, IMAGE_HEIGHT)}))
        camera.start()

        # Initialize ORB detector
        orb = cv2.ORB_create(20)

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        while(1):
            image = camera.capture_array()

            # Load the image and convery to grayscale
            frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            img_kp = keypoints(frame_gray)
            cv2.imshow('Keypoints', visualize_fps(img_kp, fps))
            key = cv2.waitKey(1) & 0xFF

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





# in case the first one doesnt work