import cv2
import numpy as np

# Capture video from webcam
cap = cv2.VideoCapture(0)

# Define lower and upper bounds for orange color in HSV
orange_lower = np.array([5, 100, 100])
orange_upper = np.array([15, 255, 255])

while True:
    print("Capturing frame")
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

    # Bitwise-AND mask and original image
    orange_object = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert the image to grayscale
    gray = cv2.cvtColor(orange_object, cv2.COLOR_BGR2GRAY)

    # Find contours in the grayscale image
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around the orange objects
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
