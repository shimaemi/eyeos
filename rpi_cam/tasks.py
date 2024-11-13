from picamera2 import Picamera2
import cv2
from ultralytics import YOLO

# Load the YOLOv11 model
model = YOLO("yolo11n.pt")

# Initialize the camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)})
picam2.configure(config)
picam2.start()

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()

    # Convert frame from 4 channels (RGBA) to 3 channels (RGB)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    # Run YOLOv11 inference on the frame
    results = model(frame)

    # Draw bounding boxes and labels on the frame
    for result in results[0].boxes:  # Access the bounding boxes directly
        x1, y1, x2, y2 = result.xyxy[0]
        confidence = result.conf[0]
        class_id = result.cls[0]
        label = model.names[int(class_id)]
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {confidence:.2f}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('YOLOv11 Real-Time Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
picam2.stop()
cv2.destroyAllWindows()
