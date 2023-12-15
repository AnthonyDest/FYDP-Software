import cv2
import apriltag
import math
import numpy as np

def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized

# Initialize the camera capture
# camera = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust as needed
image_name = "JaredHD.jpg"
image = cv2.imread(image_name)
# image

frame = image_resize(image, height = 800)

# width = 500
# height =  800
# image = cv2.resize(image, (width, height))
# frame = image
cv2.imshow("HSV-2", frame)

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Initialize the AprilTag detector with a specific family
detector = apriltag.CommonDetector() # You can choose a different family if needed

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

while True:
    # ret, frame = camera.read()

    # if not ret:
    #     print("Failed to capture a frame")
    #     break

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
        x_distance_str = f"X: {abs(x_distance):.2f} cm"
        y_distance_str = f"Y: {abs(y_distance):.2f} cm"
        z_distance_str = f"Z: {z_distance:.2f} cm"

        cv2.putText(frame, x_distance_str, (x_coord, y_coord + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, y_distance_str, (x_coord, y_coord + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, z_distance_str, (x_coord, y_coord + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        corners = detection.corners

        # Convert the corners to integers
        corners = corners.round().astype(int)

        # Draw a rectangle around the detected AprilTag
        cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        

    # Display the frame
    cv2.imshow('AprilTag Detection', frame)
    break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.imwrite('FunnyAprilTag4.jpg', frame)

# Release the camera and close the OpenCV window
# camera.release()
cv2.destroyAllWindows()
