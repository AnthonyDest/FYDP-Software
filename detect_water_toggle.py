import cv2
import numpy as np
from scipy.stats import norm

def select_roi(image):
    # Specify ROI coordinates
    roi_top_left = (225, 225)  # (x, y) coordinates of the top-left corner of the ROI
    roi_bottom_right = (400, 300)  # (x, y) coordinates of the bottom-right corner of the ROI

    # Crop the image to the specified ROI
    roi = image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]

    # Convert the ROI to HSV color space
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

    return low_snow, up_snow

# Load the image
image = cv2.imread("ice_image.jpg")

# Resize the image
width = 500
height = 300
image = cv2.resize(image, (width, height))

# Apply Gaussian blur to the mask
blur = cv2.GaussianBlur(image, (25, 25), 0)

# Convert the entire image to HSV color space
hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
cv2.imshow("AAA", hsv)
hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

cv2.imshow("BBB", hsv)
ccc = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow("CCC", ccc)
# Define whether to use ROI or not
use_roi = False  # Set this to True to use ROI, or False to analyze the entire image

if use_roi:
    low_ice, high_ice = select_roi(image)
else:
    # Manually specify the HSV low and high values
    # low_snow = np.array([20.06, 0.81, 193.67])
    # # up_snow = np.array([60.40, 3.13, 203.60])
    # low_snow = np.array([21.26, 0.86, 191.35])
    # up_snow = np.array([56.79, 2.76, 201.96])

    # low_snow = np.array([20.87, 1.47, 75.50])
    # up_snow = np.array([190.88, 15.38, 207.52])
    low_ice = np.array([-64.26, -2.85, 183.56])
    high_ice = np.array([112.46, 5.05, 208.30])

    # values of snow to be ignored below
    # low_snow = np.array([3.02, 0.04, 221.25])
    # up_snow = np.array([55.09, 1.64, 231.37])
    low_snow = np.array([0.00, 0.00, 194.54])
    high_snow = np.array([0.00, 0.00, 241.45])


# Create a mask for the snow color range
mask = cv2.inRange(hsv, low_ice, high_ice)
mask_snow = cv2.inRange(hsv, low_snow, high_snow)
# cv2.imshow("Current Mask A", mask)

kernel = np.ones((10, 10), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask_snow = cv2.morphologyEx(mask_snow, cv2.MORPH_CLOSE, kernel)
# cv2.imshow("Current Mask B", mask)


# Apply Gaussian blur to the mask
# mask = cv2.GaussianBlur(mask, (15, 15), 0)
# cv2.imshow("Current Mask", mask)

# Apply morphological operations to the mask (dilation and erosion) to show only if a large area around each color is the same color
# kernel = np.ones((1, 1), np.uint8)
# mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Apply Gaussian blur to the mask
# mask = cv2.GaussianBlur(mask, (15, 15), 0)

# Apply the mask to the entire image
res = cv2.bitwise_and(image, image, mask=mask)

remove_snow = cv2.bitwise_not(mask_snow)
cv2.imshow("Removed Snow", remove_snow)

 
double_mask = cv2.bitwise_and(mask_snow, mask_snow, mask=mask)
# cv2.imshow("Double Mask", double_mask)

# Create windows for displaying images
cv2.namedWindow("Original Image")
cv2.namedWindow("Masked Image")

# Display the original image
cv2.imshow("Original Image", image)

# Display the masked image
cv2.imshow("Masked Image", res)

cv2.waitKey(0)
cv2.destroyAllWindows()
