import cv2
from time import sleep
from apriltag import Detector

# Function to display pose information on the frame
def display_pose(frame, detections):
    for detection in detections:
        # Get tag ID and pose information
        tag_id = detection[1]  # Index 1 corresponds to the tag ID
        tag_pose = detection[2]  # Index 2 corresponds to the pose information

        # Display tag ID
        cv2.putText(frame, f'Tag ID: {tag_id}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display pose information
        cv2.putText(frame, f'Pose: {tag_pose}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame with pose information
    cv2.imshow('AprilTag Pose', frame)

# Initialize USB camera
camera = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust if needed

# Initialize AprilTag detector
detector = Detector()

# Main loop for capturing and processing frames
while True:
    # Capture a single frame
    _, frame = camera.read()

    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the grayscale frame
    detections = detector.detect(gray_frame)

    # Display pose information on the frame
    display_pose(frame, detections)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera
camera.release()
cv2.destroyAllWindows()
