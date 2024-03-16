import cv2
import numpy as np


def nothing(x):
    pass


# Create a window for trackbars
cv2.namedWindow("Trackbars")

# Create trackbars for color selection
cv2.createTrackbar("Low_H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("Low_S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Low_V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("High_H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("High_S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High_V", "Trackbars", 255, 255, nothing)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current positions of all trackbars
    low_h = cv2.getTrackbarPos("Low_H", "Trackbars")
    low_s = cv2.getTrackbarPos("Low_S", "Trackbars")
    low_v = cv2.getTrackbarPos("Low_V", "Trackbars")
    high_h = cv2.getTrackbarPos("High_H", "Trackbars")
    high_s = cv2.getTrackbarPos("High_S", "Trackbars")
    high_v = cv2.getTrackbarPos("High_V", "Trackbars")

    # Define range of color in HSV
    lower_color = np.array([low_h, low_s, low_v])
    upper_color = np.array([high_h, high_s, high_v])

    # Threshold the HSV image to get only the color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", res)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
