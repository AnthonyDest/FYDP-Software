import cv2
import numpy as np
from scipy.stats import norm

def select_roi(image):
    # Specify ROI coordinates
    roi_top_left = (225, 225)  # (x, y) coordinates of the top-left corner of the ROI
    roi_bottom_right = (400, 300)  # (x, y) coordinates of the bottom-right corner of the ROI

    # Crop the image to the specified ROI
    roi = image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]

    # Calculate the average intensity in the ROI
    avg_intensity = np.mean(roi)

    # Calculate the z-score for the desired confidence level
    confidence_level = 0.98  # For a 98% confidence level
    z_score = norm.ppf((1 + confidence_level) / 2)

    # Calculate the tolerance based on the z-score and standard deviation
    intensity_tolerance = z_score * np.std(roi)

    # Calculate the lower and upper bounds for the intensity range
    low_intensity = avg_intensity - intensity_tolerance
    up_intensity = avg_intensity + intensity_tolerance

    return low_intensity, up_intensity

# Load the image
image = cv2.imread("ice_image.jpg", cv2.IMREAD_GRAYSCALE)

# Resize the image
width = 500
height = 300
image = cv2.resize(image, (width, height))

# Define whether to use ROI or not
use_roi = False  # Set this to True to use ROI, or False to analyze the entire image

if use_roi:
    low_intensity, up_intensity = select_roi(image)
else:
    # Manually specify the grayscale intensity low and high values
    low_intensity = 195.59  # Adjust this value as needed
    up_intensity = 199.81  # Adjust this value as needed

# Create a mask for the grayscale intensity range
mask = cv2.inRange(image, low_intensity, up_intensity)

# Apply morphological operations to the mask (dilation and erosion) to show only if a large area around each intensity is the same intensity
# kernel = np.ones((1, 1), np.uint8)
# mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Apply Gaussian blur to the mask
# mask = cv2.GaussianBlur(mask, (15, 15), 0)

# Apply the mask to the entire image
res = cv2.bitwise_and(image, image, mask=mask)

# Create windows for displaying images
cv2.namedWindow("Original Image")
cv2.namedWindow("Masked Image")

# Display the original image
cv2.imshow("Original Image", image)

# Display the masked image
cv2.imshow("Masked Image", res)

cv2.waitKey(0)
cv2.destroyAllWindows()
