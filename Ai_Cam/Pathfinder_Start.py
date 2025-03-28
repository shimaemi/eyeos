import argparse
import sys
import cv2
import time
import numpy as np

from functools import lru_cache
from speech_announcer import SpeechAnnouncer

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics,
                                      postprocess_nanodet_detection)

# Initialize the SpeechAnnouncer class
speaker = SpeechAnnouncer()

# Global variables
last_detections = []
last_results = None
picam2 = None
imx500 = None
intrinsics = None
args = None


class Detection:
    def __init__(self, coords, category, conf, metadata):
        """Stores detection information including bounding box coordinates.
        
        Args:
            coords: Raw coordinates from inference output
            category: Detected object class ID
            conf: Confidence score (0-1)
            metadata: Camera metadata for coordinate conversion
        """
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


def parse_detections(metadata: dict):
    """Converts raw inference output into Detection objects.
    
    Args:
        metadata: Dictionary containing inference results
        
    Returns:
        List of Detection objects that meet confidence threshold
    """
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


    labels = get_labels()
    last_detections = [
        {
            "label": labels[int(detection.category)],
            "box": detection.box,
            "conf": detection.conf
        }
        for detection in last_detections
        if detection.conf > args.threshold
    ]

    return last_detections


@lru_cache
def get_labels():
    """Cached label loader that filters out empty/dash labels.
    
    Returns:
        List of label strings corresponding to class IDs
    """
    labels = intrinsics.labels
    if intrinsics.ignore_dash_labels:
        labels = [label for label in labels if label and label != "-"]
    return labels


def draw_detections(request, stream="main"):
    """Draws bounding boxes and labels on the camera feed.
    
    Args:
        request: Camera frame request object
        stream: Stream name to draw on (default 'main')
    """
    global last_results
    detections = last_results
    if detections is None:
        return
    labels = get_labels()
    
    with MappedArray(request, stream) as m:
        img_width = m.array.shape[1]  # Get the width of the image

        for detection in detections:
            x, y, w, h = detection.box
            object_center_x = x + w // 2  # Calculate center x of object

            # Determine position category
            if object_center_x < img_width // 3:
                position = "Left"
            elif object_center_x > (2 * img_width) // 3:
                position = "Right"
            else:
                position = "Middle"

            # Create label with object name, confidence, and position
            label = f"{labels[int(detection.category)]} ({detection.conf:.2f}) - {position}"

            # Calculate text size and position
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = x + 5
            text_y = y + 15

            # Create overlay for text background
            overlay = m.array.copy()
            cv2.rectangle(overlay, (text_x, text_y - text_height),
                          (text_x + text_width, text_y + baseline),
                          (255, 255, 255), cv2.FILLED)

            alpha = 0.30
            cv2.addWeighted(overlay, alpha, m.array, 1 - alpha, 0, m.array)

            # Draw text on image
            cv2.putText(m.array, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Draw bounding box around object
            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, help="Path of the model",
                        default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
    parser.add_argument("--fps", type=int, help="Frames per second")
    parser.add_argument("--bbox-normalization", action=argparse.BooleanOptionalAction, help="Normalize bbox")
    parser.add_argument("--bbox-order", choices=["yx", "xy"], default="yx",
                        help="Set bbox order yx -> (y0, x0, y1, x1) xy -> (x0, y0, x1, y1)")
    parser.add_argument("--threshold", type=float, default=0.55, help="Detection threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="Set iou threshold")
    parser.add_argument("--max-detections", type=int, default=10, help="Set max detections")
    parser.add_argument("--ignore-dash-labels", action=argparse.BooleanOptionalAction, help="Remove '-' labels ")
    parser.add_argument("--postprocess", choices=["", "nanodet"],
                        default=None, help="Run post process of type")
    parser.add_argument("-r", "--preserve-aspect-ratio", action=argparse.BooleanOptionalAction,
                        help="preserve the pixel aspect ratio of the input tensor")
    parser.add_argument("--labels", type=str,
                        help="Path to the labels file")
    parser.add_argument("--print-intrinsics", action="store_true",
                        help="Print JSON network_intrinsics then exit")
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()

    # This must be called before instantiation of Picamera2
    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        print("Network is not an object detection task", file=sys.stderr)
        exit()

    # Override intrinsics from args
    for key, value in vars(args).items():
        if key == 'labels' and value is not None:
            with open(value, 'r') as f:
                intrinsics.labels = f.read().splitlines()
        elif hasattr(intrinsics, key) and value is not None:
            setattr(intrinsics, key, value)

    # Defaults
    if intrinsics.labels is None:
        with open("assets/coco_labels.txt", "r") as f:
            intrinsics.labels = f.read().splitlines()
    intrinsics.update_with_defaults()

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
            # Capture and parse detections
            last_results = parse_detections(picam2.capture_metadata())
            
            # Only announce if new detections exist
            if last_results:
                for detection in last_results:
                    try:
                        # Calculate position
                        x, _, w, _ = detection["box"]
                        img_width = picam2.camera_configuration()['main']['size'][0] if picam2 else 1280
                        position = ("left" if x + w//2 < img_width//3 else
                                  "right" if x + w//2 > 2*img_width//3 else "center")
                        
                        # Announce with error handling
                        speaker.announce(f"{detection['label']} detected on the {position}")
                    except Exception as e:
                        print(f"Speech error: {e}")
            
            # Maintain CPU-friendly delay
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        picam2.stop()
        print("Camera stopped gracefully")