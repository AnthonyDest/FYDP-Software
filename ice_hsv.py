import cv2
import numpy as np
from scipy.stats import norm

# Global variables to store ROI coordinates and list of selected ROIs
rois = []
selecting_roi = False

def on_mouse(event, x, y, flags, param):
    global rois, selecting_roi

    if event == cv2.EVENT_LBUTTONDOWN:
        rois.append((x, y))
        selecting_roi = True

# Create a window and set the mouse callback
cv2.namedWindow("Select ROIs")
cv2.setMouseCallback("Select ROIs", on_mouse)

# Load the image
image = cv2.imread("ice_image.jpg")

# Resize the image
width = 500
height = 300
image = cv2.resize(image, (width, height))

while True:
    if selecting_roi:
        roi_image = image.copy()
        for i in range(0, len(rois), 2):
            if i + 1 < len(rois):
                top_left = (rois[i])
                bottom_right = (rois[i + 1])
                cv2.rectangle(roi_image, top_left, bottom_right, (0, 255, 0), 2)
        cv2.imshow("Select ROIs", roi_image)
    else:
        cv2.imshow("Select ROIs", image)

    key = cv2.waitKey(1)

    if key == ord('r'):
        # Press 'r' key to confirm and select another ROI
        selecting_roi = False
        rois = []  # Clear the selected ROI
    elif key == 13:
        # Press 'Enter' key to finish ROI selection
        break

# List to store HSV statistics for all selected windows
roi_hsv_stats = []

# Calculate HSV statistics for each selected ROI
for i in range(0, len(rois), 2):
    if i + 1 < len(rois):
        top_left = (rois[i])
        bottom_right = (rois[i + 1])
        roi = image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        avg_hue = np.mean(hsv_roi[:, :, 0])
        avg_saturation = np.mean(hsv_roi[:, :, 1])
        avg_value = np.mean(hsv_roi[:, :, 2])

        roi_hsv_stats.append(
            {
                "Avg Hue": avg_hue,
                "Avg Saturation": avg_saturation,
                "Avg Value": avg_value,
            }
        )

# Calculate the cumulative mean
cumulative_mean_hue = np.mean([stats["Avg Hue"] for stats in roi_hsv_stats])
cumulative_mean_saturation = np.mean([stats["Avg Saturation"] for stats in roi_hsv_stats])
cumulative_mean_value = np.mean([stats["Avg Value"] for stats in roi_hsv_stats])

# Calculate the standard deviation
std_dev_hue = np.std([stats["Avg Hue"] for stats in roi_hsv_stats])
std_dev_saturation = np.std([stats["Avg Saturation"] for stats in roi_hsv_stats])
std_dev_value = np.std([stats["Avg Value"] for stats in roi_hsv_stats])

# Calculate the z-score for the desired confidence level (95%)
confidence_level = 0.95
z_score = norm.ppf((1 + confidence_level) / 2)

# Calculate the tolerance based on the z-score and standard deviation
hue_tolerance = z_score * (std_dev_hue / np.sqrt(len(roi_hsv_stats)))
saturation_tolerance = z_score * (std_dev_saturation / np.sqrt(len(roi_hsv_stats)))
value_tolerance = z_score * (std_dev_value / np.sqrt(len(roi_hsv_stats)))

# Calculate the lower and upper bounds
low_hue = cumulative_mean_hue - hue_tolerance
low_saturation = cumulative_mean_saturation - saturation_tolerance
low_value = cumulative_mean_value - value_tolerance

up_hue = cumulative_mean_hue + hue_tolerance
up_saturation = cumulative_mean_saturation + saturation_tolerance
up_value = cumulative_mean_value + value_tolerance

print(f"Lower Bounds: Hue={low_hue:.2f}, Saturation={low_saturation:.2f}, Value={low_value:.2f}")
print(f"Upper Bounds: Hue={up_hue:.2f}, Saturation={up_saturation:.2f}, Value={up_value:.2f}")

# Format for copying and pasting into HSV low_snow and up_snow values
print(f"low_snow = np.array([{low_hue:.2f}, {low_saturation:.2f}, {low_value:.2f}])")
print(f"up_snow = np.array([{up_hue:.2f}, {up_saturation:.2f}, {up_value:.2f}])")

cv2.waitKey(0)
cv2.destroyAllWindows()
