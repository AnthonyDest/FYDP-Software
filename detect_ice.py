import cv2
import numpy as np
import ice_hsv_autotune_select_window

# Load the image
image_name = "ice_image1.jpg"
image = cv2.imread(image_name)

# Resize the image
width = 500
height = 300
image = cv2.resize(image, (width, height))

# Define whether to manually select ROI
use_roi = False 

# Select ROI
if use_roi:
    low_ice, high_ice = ice_hsv_autotune_select_window.select_roi(image_name)
    low_snow = np.array([0.00, 0.00, 194.54])
    high_snow = np.array([0.00, 0.00, 241.45])

# Manually specify the HSV low and high values
else:
    # Manually specify the HSV low and high values

    # image 1 - good value
    low_ice = np.array([0.00, 0.00, 172.00])
    high_ice = np.array([75.00, 3.00, 197.00])


    # image 2 - good value
    # low_ice = np.array([85.55, 23.48, 145.25])
    # high_ice = np.array([128.33, 35.22, 217.88])

    # image 3 - good value
    # low_ice = np.array([101.00, 27.00, 172.00])
    # high_ice = np.array([110.00, 33.00, 187.00])

    # values of snow to be ignored below
    # low_snow = np.array([3.02, 0.04, 221.25])
    # up_snow = np.array([55.09, 1.64, 231.37])
    low_snow = np.array([0.00, 0.00, 194.54])
    high_snow = np.array([0.00, 0.00, 241.45])

# Apply Gaussian blur to the mask
blur = cv2.GaussianBlur(image, (5, 5), 0)
cv2.imshow("Blur-1", blur)

# Convert the entire image to HSV color space
hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
cv2.imshow("HSV-2", hsv)

# Create a mask for the snow color range
mask = cv2.inRange(hsv, low_ice, high_ice)
cv2.imshow("In HSV Range mask-3", mask)

# mask_snow = cv2.inRange(hsv, low_snow, high_snow)
# cv2.imshow("Current Mask A", mask)

# group each section into X by Y groups of pixels
kernel = np.ones((20, 20), np.uint8)
group_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
cv2.imshow("Group pixel mask-4", group_mask)


# kernel = np.ones((5, 5), np.uint8)
# smoothed_mask = cv2.morphologyEx(group_mask, cv2.MORPH_CLOSE, kernel)
# cv2.imshow("Blur-5", smoothed_mask)

# second blur to smoothen group mask
# blur_second = cv2.GaussianBlur(group_mask, (15, 15), 0)
# # smoothed_mask = cv2.GaussianBlur(closed_mask, (15, 15), 0)
# cv2.imshow("Blur-5", blur_second)

# mask_snow = cv2.morphologyEx(mask_snow, cv2.MORPH_CLOSE, kernel)
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
res = cv2.bitwise_and(image, image, mask=group_mask)

# remove_snow = cv2.bitwise_not(mask_snow)
# cv2.imshow("Removed Snow", remove_snow)

 
# double_mask = cv2.bitwise_and(mask_snow, mask_snow, mask=mask)
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

