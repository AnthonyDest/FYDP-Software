import cv2
import numpy as np
from scipy.stats import norm

# Global variables to store ROI coordinates
roi_top_left = None
roi_bottom_right = None
selecting_roi = False

def on_mouse(event, x, y, flags, param):
    global roi_top_left, roi_bottom_right, selecting_roi
    
    if event == cv2.EVENT_LBUTTONDOWN:
        roi_top_left = (x, y)
        selecting_roi = True
    elif event == cv2.EVENT_LBUTTONUP:
        roi_bottom_right = (x, y)
        selecting_roi = False

# Create a window and set the mouse callback
cv2.namedWindow("Select ROI")
cv2.setMouseCallback("Select ROI", on_mouse)

# Load the image
image = cv2.imread("ice_image.jpg")

# Resize the image
width = 500
height = 300
image = cv2.resize(image, (width, height))

while True:
    if roi_top_left and roi_bottom_right:
        # Draw a rectangle for ROI selection
        roi_image = image.copy()
        cv2.rectangle(roi_image, roi_top_left, roi_bottom_right, (0, 255, 0), 2)
        cv2.imshow("Select ROI", roi_image)
    else:
        cv2.imshow("Select ROI", image)  # Display the original image

    key = cv2.waitKey(1)

    if key == 13:  # Press 'Enter' key to calculate HSV range and apply the mask
        break

if roi_top_left and roi_bottom_right:
    # Crop the image to the selected ROI
    roi = image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]

    # Convert the cropped region to HSV color space
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Calculate the average HSV values in the ROI
    avg_hue = np.mean(hsv_roi[:, :, 0])
    avg_saturation = np.mean(hsv_roi[:, :, 1])
    avg_value = np.mean(hsv_roi[:, :, 2])

    # Calculate the z-score for the desired confidence level
    confidence_level = 0.98  # For a 98% confidence level
    z_score = norm.ppf((1 + confidence_level) / 2)

    # Calculate the tolerance based on the z-score and standard deviation
    hue_tolerance = z_score * np.std(hsv_roi[:, :, 0])
    saturation_tolerance = z_score * np.std(hsv_roi[:, :, 1])
    value_tolerance = z_score * np.std(hsv_roi[:, :, 2])

    # Calculate the lower and upper bounds for the HSV range
    low_snow = np.array([avg_hue - hue_tolerance, avg_saturation - saturation_tolerance, avg_value - value_tolerance])
    up_snow = np.array([avg_hue + hue_tolerance, avg_saturation + saturation_tolerance, avg_value + value_tolerance])

    # Convert the whole image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask for the snow color range
    mask = cv2.inRange(hsv, low_snow, up_snow)

    # Apply morphological operations to the mask (dilation and erosion) to show only if a large area around each color is the same color
    kernel = np.ones((10, 10), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Apply Gaussian blur to the mask
    mask = cv2.GaussianBlur(mask, (15, 15), 0)

    # Apply the mask to the entire image
    res = cv2.bitwise_and(image, image, mask=mask)
    print(low_snow)
    print(up_snow)
    print(f"low_snow = np.array([{low_snow[0]:.2f}, {low_snow[1]:.2f}, {low_snow[2]:.2f}])")
    print(f"up_snow = np.array([{up_snow[0]:.2f}, {up_snow[1]:.2f}, {up_snow[2]:.2f}]))")
    # Display the mask applied to the entire image
    cv2.imshow("Image with Snow Mask", res)

cv2.waitKey(0)
cv2.destroyAllWindows()
