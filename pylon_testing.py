import cv2
import numpy as np


# Function to read a still image
def read_image(image_path):
    frame = cv2.imread(image_path)
    return frame


# Function to detect orange color
def detect_orange(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 150, 100])
    upper_orange = np.array([15, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Get the largest contour
        c = max(contours, key=cv2.contourArea)
        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(c)
        return x, y, w, h
    else:
        return None


# Function to draw bounding box around the pylon
def draw_box(frame, x, y, w, h):
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


def compute_box_position(frame, x, w):
    # Get the center x-coordinate of the bounding box
    box_center_x = x + w // 2
    # Get the width of the image
    image_width = frame.shape[1]
    # Compare the position of the box relative to the center of the image

    # zz give threshold for the center of the image
    if box_center_x < image_width / 2:
        return "pylon on left, steer left"
    else:
        return "pylon on right, steer right"


# Main function
def main():
    # Path to your still image
    image_path = "pylon_right.jpg"
    image_path = "pylon_center.jpg"
    frame = read_image(image_path)
    orange_position = detect_orange(frame)
    if orange_position is not None:
        x, y, w, h = orange_position
        draw_box(frame, x, y, w, h)
    cv2.imshow("Frame", frame)
    print(compute_box_position(frame, x, w))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
