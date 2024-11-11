import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
from tf_luna import TFLuna

# Initialize camera 30 fps
frames = 30

# detects objects in an image using the YOLO library and a set of pre-trained objects from the COCO database;
# a set of 80 classes is listed in "coco.names" and pre-trained weights are stored in "yolov3.weights"
def detect_objects(img, bBoxes, conf_threshold, nms_threshold, base_path, classes_file, model_configuration, model_weights, b_vis):
    # load class names from file
    classes = []
    with open(classes_file, 'r') as ifs:
        classes = [line.strip() for line in ifs.readlines()]
    
    # load neural network
    net = cv2.dnn.readNetFromDarknet(model_configuration, model_weights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
    # generate 4D blob from input image
    blob = cv2.dnn.blobFromImage(img, 1/255.0, (416, 416), (0, 0, 0), False, False)
    
    # Get names of output layers
    out_layers = net.getUnconnectedOutLayers()  # get indices of output layers
    layers_names = net.getLayerNames()  # get names of all layers in the network
    
    names = [layers_names[i - 1] for i in out_layers]
    
    # invoke forward propagation through network
    net.setInput(blob)
    net_output = net.forward(names)
    
    # Scan through all bounding boxes and keep only the ones with high confidence
    class_ids, confidences, boxes = [], [], []
    for output in net_output:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            if confidence > conf_threshold:
                box = detection[0:4] * np.array([img.shape[1], img.shape[0], img.shape[1], img.shape[0]])
                (cx, cy, width, height) = box.astype("int")
                x = int(cx - (width / 2))  # left
                y = int(cy - (height / 2))  # top
                
                boxes.append((x, y, width, height))
                class_ids.append(class_id)
                confidences.append(float(confidence))
    
    # perform non-maxima suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    for i in indices:
        i = i[0]
        bBox = BoundingBox()
        bBox.roi = boxes[i]
        bBox.classID = class_ids[i]
        bBox.confidence = confidences[i]
        bBox.boxID = len(bBoxes)  # zero-based unique identifier for this bounding box
        
        bBoxes.append(bBox)

    # show results
    if bVis:
        visImg = img.copy()
        for bbox in bBoxes:
            # Draw rectangle displaying the bounding box
            top = bbox.roi.y
            left = bbox.roi.x
            width = bbox.roi.width
            height = bbox.roi.height
            cv2.rectangle(visImg, (left, top), (left + width, top + height), (0, 255, 0), 2)

            label = f"{classes[bbox.classID]}:{bbox.confidence:.2f}:{bbox.boxID}"

            # Display label at the top of the bounding box
            (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_ITALIC, 0.5, 1)
            top = max(top, label_height)
            cv2.rectangle(visImg, (left, top - round(1.5 * label_height)), (left + round(1.5 * label_width), top + baseline), (255, 255, 255), cv2.FILLED)
            cv2.putText(visImg, label, (left, top), cv2.FONT_ITALIC, 0.75, (0, 0, 0), 1)

        windowName = "Object classification"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.imshow(windowName, visImg)

        cv2.waitKey(0)  # wait for key to be pressed

if __name__ == "__main__":
    try:
        cam_init(frames)
        while(1)
            # record start time
            start_time = time.time()

            camera.capture('img1.jpg')
            detect_objects(img1.jpg, bBoxes, conf_threshold, nms_threshold, base_path, classes_file, model_configuration, model_weights, b_vis)

            # record end time
            end_time = time.time()
            # calculate FPS
            seconds = end_time - start_time
            fps = 1.0 / seconds
            print("Estimated fps:{0:0.1f}".format(fps))

    except KeyboardInterrupt:
        camera.close()
        cv2.destroyAllWindows()
        print("program interrupted by the user")