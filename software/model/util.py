from ultralytics import YOLO
import cv2

# Load the YOLO model
model_path = "best.pt"  # Update with the path to your model
model = YOLO(model_path)

# Set the input video source (can be a file or webcam)
input_source = "geometry.mp4"  # Replace with your video file path or use 0 for webcam

# Open the video source
cap = cv2.VideoCapture(input_source)

# Check if the video source opened successfully
if not cap.isOpened():
    print("Error: Cannot open video source.")
    exit()

# Get video properties for saving output
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))
output_path = "output_video.avi"  # Output video file path
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI files
out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

name = {'0':'Circle', '1':'Rectangle', '2':'Triangle'}

# Process the video frame by frame
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform object detection on the current frame
    results = model.predict(source=frame, conf=0.5, save=False, stream=True)

    # Annotate the frame with detection results
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()  # Get bounding box coordinates
        confidences = result.boxes.conf.cpu().numpy()  # Get confidence scores
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # Get class IDs

        for box, confidence, class_id in zip(boxes, confidences, class_ids):
            # Unpack box coordinates
            xmin, ymin, xmax, ymax = map(int, box)
            label = f"{name[model.names[class_id]]} {confidence:.2f}"  # Get class name and confidence

            # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Save the annotated frame to the output video
    out.write(frame)

    # Display the frame (optional)
    # cv2.imshow("Detection Results", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Annotated video saved at {output_path}")
