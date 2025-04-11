import argparse
import sys
import cv2
import time
import numpy as np

from functools import lru_cache
from speech_announcer import SpeechAnnouncer
from haptic_vibration import HapticController
from lidar_sensor import TFLuna

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics,
                                      postprocess_nanodet_detection)

# Initialize the other classes
speaker = SpeechAnnouncer()
haptic = HapticController(left_pin=17, right_pin=18)
lidar_sensor = TFLuna()

# Global variables
last_detections = []
last_results = None
picam2 = None
imx500 = None
intrinsics = None
args = None
wall_distance_threshold_cm = 100  



def check_lidar_proximity():
    """Check lidar distance and return it if valid"""
    distance = lidar_sensor.read_distance()
    if distance is not None:
        # Convert to meters or any other preferred unit
        distance_in_meters = lidar_sensor.convert_distance(distance, 'm')
        return distance_in_meters
    return None

def detect_wall_using_lidar(lidar_distance, threshold=0.5):
    """Detect wall using lidar distance"""
    if lidar_distance is not None and lidar_distance < threshold:  # Threshold is 0.5 meters here
        return True
    return False


class Detection:
    def __init__(self, coords, category, conf, metadata):
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


def parse_detections(metadata: dict):
    global last_detections
    
    bbox_normalization = intrinsics.bbox_normalization
    bbox_order = intrinsics.bbox_order
    threshold = args.threshold
    iou = args.iou
    max_detections = args.max_detections

    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    input_w, input_h = imx500.get_input_size()
    if np_outputs is None:
        return last_detections
    if intrinsics.postprocess == "nanodet":
        boxes, scores, classes = \
            postprocess_nanodet_detection(outputs=np_outputs[0], conf=threshold, iou_thres=iou,
                                          max_out_dets=max_detections)[0]
        from picamera2.devices.imx500.postprocess import scale_boxes
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
        if bbox_normalization:
            boxes = boxes / input_h

        if bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]
        boxes = np.array_split(boxes, 4, axis=1)
        boxes = zip(*boxes)

    last_detections = [
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]
    return last_detections


@lru_cache
def get_labels():
    labels = intrinsics.labels
    if intrinsics.ignore_dash_labels:
        labels = [label for label in labels if label and label != "-"]
    return labels


def get_position_and_proximity(detection, img_width, img_height):
    """Return position (left/center/right) and proximity (incoming/close/very close)"""
    x, y, w, h = detection.box
    object_center_x = x + w // 2
    box_area = w * h
    img_area = img_width * img_height
    area_ratio = box_area / img_area

    # Position
    if object_center_x < img_width // 3:
        position = "left"
    elif object_center_x > (2 * img_width) // 3:
        position = "right"
    else:
        position = "ahead"

    # Proximity thresholds
    proximity = None
    if area_ratio > 0.2:
        proximity = "very close"
    elif area_ratio > 0.08:
        proximity = "close"
    elif area_ratio > 0:
        proximity = "incoming"

    return position, proximity


def draw_detections(request, stream="main"):
    detections = last_results
    if detections is None:
        return
    labels = get_labels()
    
    with MappedArray(request, stream) as m:
        img_width = m.array.shape[1]
        img_height = m.array.shape[0]

        for detection in detections:
            x, y, w, h = detection.box
            position, proximity = get_position_and_proximity(detection, img_width, img_height)
            label = f"{labels[int(detection.category)]} ({detection.conf:.2f}) - {position.capitalize()}"

            # Add "incoming", "close", or "very close" only if proximity is within range
            if proximity:
                label += f" {proximity}"

            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = x + 5
            text_y = y + 15

            overlay = m.array.copy()
            cv2.rectangle(overlay, (text_x, text_y - text_height),
                          (text_x + text_width, text_y + baseline),
                          (255, 255, 255), cv2.FILLED)

            alpha = 0.30
            cv2.addWeighted(overlay, alpha, m.array, 1 - alpha, 0, m.array)

            cv2.putText(m.array, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str,
                        default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
    parser.add_argument("--fps", type=int)
    parser.add_argument("--bbox-normalization", action=argparse.BooleanOptionalAction)
    parser.add_argument("--bbox-order", choices=["yx", "xy"], default="yx")
    parser.add_argument("--threshold", type=float, default=0.65)
    parser.add_argument("--iou", type=float, default=0.65)
    parser.add_argument("--max-detections", type=int, default=10)
    parser.add_argument("--ignore-dash-labels", action=argparse.BooleanOptionalAction)
    parser.add_argument("--postprocess", choices=["", "nanodet"], default=None)
    parser.add_argument("-r", "--preserve-aspect-ratio", action=argparse.BooleanOptionalAction)
    parser.add_argument("--labels", type=str)
    parser.add_argument("--print-intrinsics", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()

    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics

    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        print("Network is not an object detection task", file=sys.stderr)
        exit()

    for key, value in vars(args).items():
        if key == 'labels' and value is not None:
            with open(value, 'r') as f:
                intrinsics.labels = f.read().splitlines()
        elif hasattr(intrinsics, key) and value is not None:
            setattr(intrinsics, key, value)

    if args.print_intrinsics:
        print(intrinsics)
        exit()

    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(controls={"FrameRate": intrinsics.inference_rate}, buffer_count=12)

    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=True)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    last_results = None
    picam2.pre_callback = draw_detections

    try:
        while True:
            # Capture detections and lidar distance
            last_results = parse_detections(picam2.capture_metadata())
            lidar_distance = check_lidar_proximity()

            # If no object detected by camera and lidar reads something close
            if not last_results and detect_wall_using_lidar(lidar_distance):
                print(f"Wall detected using lidar! Distance: {lidar_distance:.2f} meters")
                speaker.announce("Wall detected ahead")
                haptic.activate_left(intensity=100, duration=1)  # Stronger haptic feedback when a wall is detected

            if last_results:
                labels = get_labels()
                img_width = picam2.camera_configuration()['main']['size'][0] if picam2 else 1280
                img_height = picam2.camera_configuration()['main']['size'][1] if picam2 else 720
            
                for detection in last_results:
                    try:
                        label = labels[int(detection.category)]
                        position, proximity = get_position_and_proximity(detection, img_width, img_height)

                        # Announce object detections
                        if proximity:  # Announce only if proximity is "incoming", "close", or "very close"
                            speaker.announce(f"{label} {proximity} {position}")

                        if position == "left":
                            haptic.activate_left(intensity=100, duration=0.3)
                        elif position == "right":
                            haptic.activate_right(intensity=100, duration=0.3)

                    except Exception as e:
                        print(f"Speech error: {e}")

            else:
                distance = lidar_sensor.read_distance()
                if distance is not None and distance < wall_distance_threshold_cm:
                    speaker.announce("Wall detected ahead")
                    haptic.activate_left(intensity=100, duration=1)  # Stronger haptic feedback when a wall is detected
                    haptic.activate_right(intensity=100, duration=1)  # Stronger haptic feedback when a wall is detected                                    

            time.sleep(0.01)
        
    except KeyboardInterrupt:
        picam2.stop()
        print("Stopping the program")

    finally:
        picam2.close()
        haptic.cleanup()
        lidar_sensor.close()
        print("Cleanup completed")
