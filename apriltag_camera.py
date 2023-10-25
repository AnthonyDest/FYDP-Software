import cv2
import apriltag

# Initialize the camera capture
camera = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust as needed

# Initialize the AprilTag detector with a specific family
detector = apriltag.Detector()  # You can choose a different family if needed

while True:
    ret, frame = camera.read()

    if not ret:
        print("Failed to capture a frame")
        break

    # Convert the frame to grayscale for AprilTag detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the grayscale frame
    detections = detector.detect(gray)

    for detection in detections:
        # Get the corners of the detected AprilTag
        corners = detection.corners

        # Convert the corners to integers
        corners = corners.round().astype(int)

        # Draw a rectangle around the detected AprilTag
        cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    # Display the frame
    cv2.imshow('AprilTag Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
camera.release()
cv2.destroyAllWindows()
