import cv2
import numpy as np

# Global variables to store ROI coordinates and list of selected ROIs
rois = []
selecting_roi = False

def on_mouse(event, x, y, flags, param):
    global rois, selecting_roi

    if event == cv2.EVENT_LBUTTONDOWN:
        rois.append((x, y))
        selecting_roi = True

def select_roi(image_path):
    global rois, selecting_roi

    # Create a window and set the mouse callback
    cv2.namedWindow("Select ROIs")
    cv2.setMouseCallback("Select ROIs", on_mouse)

    # Load the image
    image = cv2.imread(image_path)
    blur = cv2.GaussianBlur(image, (5, 5), 0)

    # Resize the image
    width = 500
    height = 300
    blur = cv2.resize(blur, (width, height))

    # Select image
    while True:
        # Put a box around the image
        if selecting_roi:
            roi_image = blur.copy()
            for i in range(0, len(rois), 2):
                if i + 1 < len(rois):
                    top_left = (rois[i])
                    bottom_right = (rois[i + 1])
                    cv2.rectangle(roi_image, top_left, bottom_right, (0, 255, 0), 2)
            cv2.imshow("Select ROIs", roi_image)
        else:
            cv2.imshow("Select ROIs", blur)

        key = cv2.waitKey(1)

        if key == ord('r'):
            # Press 'r' key to confirm and select another ROI
            selecting_roi = False
        elif key == 13:
            # Press 'Enter' key to finish ROI selection
            break

    # List to store HSV statistics for all selected windows
    roi_hsv_stats = []

    top_left = (rois[0])
    bottom_right = (rois[0 + 1])
    roi = blur[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    low_hue = np.percentile(hsv_roi[:, :, 0], 2)
    low_saturation = np.percentile(hsv_roi[:, :, 1], 20)
    low_value = np.percentile(hsv_roi[:, :, 2], 20)

    up_hue = np.percentile(hsv_roi[:, :, 0], 80)
    up_saturation = np.percentile(hsv_roi[:, :, 1], 80)
    up_value = np.percentile(hsv_roi[:, :, 2], 80)

    low_ice = np.array([low_hue, low_saturation, low_value])
    high_ice = np.array([up_hue, up_saturation, up_value])

    # Format for copying and pasting into HSV low_snow and up_snow values
    print(f"low_ice = np.array([{low_hue:.2f}, {low_saturation:.2f}, {low_value:.2f}])")
    print(f"high_ice = np.array([{up_hue:.2f}, {up_saturation:.2f}, {up_value:.2f}])")

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return low_ice, high_ice

if __name__ == "__main__":
    image_path = "ice_image3.jpg"
    select_roi(image_path)
