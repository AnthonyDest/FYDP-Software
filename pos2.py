import cv2
import apriltag
import math
import numpy as np
import time
# Initialize the camera capture
camera = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust as needed
record_duration  =5
# Check if the camera opened successfully
if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Initialize the AprilTag detector with a specific family
detector = apriltag.Detector()  # You can choose a different family if needed

# Known values
actual_tag_size = 14.0  # in cm^2
field_of_view_degrees = 78  # Replace with the FOV of your camera in degrees
image_width_pixels = 1920  # Replace with the actual image width in pixels
image_height_pixels = 1080

# Camera calibration parameters (replace with your values)
fx = 1000.0  # Focal length in pixels (X-axis)
fy = 1000.0  # Focal length in pixels (Y-axis)
cx = image_width_pixels / 2.0  # X-axis center
cy = image_height_pixels / 2.0  # Y-axis center

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # You can choose a different codec
output = cv2.VideoWriter('output_video2.avi', fourcc, 20.0, (image_width_pixels, image_height_pixels))

# while True:
start_time = time.time()
while (time.time() - start_time) < record_duration:
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

        # Calculate the distance to the tag
        tag_width_pixels = math.sqrt((corners[0][0] - corners[1][0])**2 + (corners[0][1] - corners[1][1])**2)
        field_of_view_radians = math.radians(field_of_view_degrees)

        # Calculate the distance along the X, Y, and Z axes
        distance = (actual_tag_size / 2) / math.tan((tag_width_pixels / 2) * (field_of_view_radians / image_width_pixels))
        
        # Calculate X, Y, and Z distances
        x_distance = (corners[0][0] - cx) * distance / fx
        y_distance = (corners[0][1] - cy) * distance / fy
        z_distance = distance  # Assuming the tag is in the same plane as the camera

        # Convert the coordinates to integers
        x_coord = int(corners[0][0])
        y_coord = int(corners[0][1])

        # Display the X, Y, and Z distances on the image
        x_distance_str = f"X: {x_distance:.2f} cm"
        y_distance_str = f"Y: {y_distance:.2f} cm"
        z_distance_str = f"Z: {z_distance:.2f} cm"

        cv2.putText(frame, x_distance_str, (x_coord, y_coord + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, y_distance_str, (x_coord, y_coord + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, z_distance_str, (x_coord, y_coord + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        corners = detection.corners

        # Convert the corners to integers
        corners = corners.round().astype(int)

        # Draw a rectangle around the detected AprilTag
        cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    # Write the frame to the output video file
    output.write(frame)

    # Display the frame
    cv2.imshow('AprilTag Detection', frame)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Release the camera, output video file, and close the OpenCV window
camera.release()
output.release()
cv2.destroyAllWindows()