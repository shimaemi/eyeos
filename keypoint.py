# test to display image keypoints
import cv2
import numpy as np
import time
from tf_luna import TFLuna
from PIL import Image

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 36)

def keypoints(image):
    # Load the image and convery to grayscale
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect key points and compute descriptors
    keypoints, descriptors = orb.detectAndCompute(img, None)
    # for x in keypoints:
        # print("({:.2f},{:.2f}) = size {:.2f} angle {:.2f}".format(x.pt[0], x.pt[1], x.size, x.angle))
    
    img_kp = cv2.drawKeypoints(img, keypoints, None,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return img_kp

if __name__ == "__main__":
    try:
        print("q to quit")

        # Initialize ORB detector
        orb = cv2.ORB_create(20)

        time.sleep(1)
        fps = 0
        # record start time
        start_time = time.time()

        while(1):
            ret, image = cap.read()
            img_kp = keypoints(image)
            cv2.imshow('Keypoints', visualize_fps(img_kp, fps))
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
        cap.release()
        cv2.destroyAllWindows()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("program interrupted by the user")