import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load a still image
image = cv2.imread("ice_image.jpg")

width = 500
height = 300

image = cv2.resize(image, (width,height))
# You don't need a loop for a single image
# while True:

# Use the image directly
# ret, orig_frame = video.read()
# if not ret:
#     video = cv2.VideoCapture("road_car_view.mp4")
#     continue

# Apply Gaussian blur to the image
frame = cv2.GaussianBlur(image, (5, 5), 0)

# cv2.imshow("frame",frame)
# Convert the frame to HSV color space
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# # Define the lower and upper bounds for the snow color range
# low_snow = np.array([0, 0, 0])
# up_snow = np.array([5, 0, 200])

# define ROI of RGB image 'img'
roi_top_left = (225, 225)  # (x, y) coordinates of the top-left corner of the ROI
roi_bottom_right = (400, 300)  # (x, y) coordinates of the bottom-right corner of the ROI

# Crop the image to the ROI
roi = image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]

# Convert the cropped region to HSV color space
hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

# Calculate the average HSV values in the ROI
avg_hue = np.mean(hsv_roi[:, :, 0])
avg_saturation = np.mean(hsv_roi[:, :, 1])
avg_value = np.mean(hsv_roi[:, :, 2])

# Draw a rectangle around the ROI
cv2.rectangle(image, roi_top_left, roi_bottom_right, (0, 255, 0), 2)

# Display the image with the ROI and average HSV values
cv2.putText(image, f'Hue: {avg_hue:.2f}', (roi_top_left[0], roi_top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
cv2.putText(image, f'Saturation: {avg_saturation:.2f}', (roi_top_left[0], roi_top_left[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
cv2.putText(image, f'Value: {avg_value:.2f}', (roi_top_left[0], roi_top_left[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image
# cv2.imshow('Image with ROI', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# convert it into HSV
hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

# # Create a mask for the snow color range
# mask = cv2.inRange(hsv, low_snow, up_snow)

# Define the lower and upper bounds for the snow color range
# low_snow = np.array([0, 0, 0])
# up_snow = np.array([5, 0, 200])
low_snow = np.array([avg_hue, avg_saturation, avg_value])
up_snow = np.array([avg_hue+5, avg_saturation+5, avg_value+5])

mask = cv2.inRange(hsv, low_snow, up_snow)


# res = cv2.bitwise_and(frame,frame, mask= mask)
cv2.imshow("mask", mask)
# cv2.imshow("res", res)
# cv2.imshow("roi", roi)


# # Apply Canny edge detection to the mask
# edges = cv2.Canny(mask, 75, 150)

# # Detect lines in the edges using the Hough Transform
# lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)

# if lines is not None:
#     for line in lines:
#         x1, y1, x2, y2 = line[0]
#         cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

# Display the modified image with detected lines

# width = 960
# height = 540


# frame = cv2.resize(frame, (width, height))                # Resize image
# mask = cv2.resize(mask, (width, height))                # Resize image

# cv2.imshow("frame", frame)
# cv2.imshow("edges", mask)
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# gray = image
# # gray = cv2.resize(gray, (width, height))  

# gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
# ret, thresh = cv2.threshold(gray, 100, 150, cv2.THRESH_BINARY)
# # cv2.imshow('Binary image', thresh)
# # cv2.imwrite("a",)

# # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE

# # B, G, R channel splitting
# blue, green, red = cv2.split(image)

# # contours, hierarchy = cv2.findContours(image=red, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

# # detect the contours on the binary image using cv2.ChAIN_APPROX_SIMPLE
# contours1, hierarchy1 = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# # draw contours on the original image for `CHAIN_APPROX_SIMPLE`

# image_copy = image.copy()
# cv2.drawContours(image_copy, contours1, -1, (0, 255, 0), 2, cv2.LINE_AA)
                                      
# # draw contours on the original image
# # image_copy = image.copy()
# cv2.drawContours(image=image_copy, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
                
# # see the results
# cv2.imshow('None approximation', image_copy)


# histogram, bin_edges = np.histogram(gray, bins=256, range=(0.0, 1.0))

# fig, ax = plt.subplots()
# plt.plot(bin_edges[0:-1], histogram)
# plt.title("Grayscale Histogram")
# plt.xlabel("grayscale value")
# plt.ylabel("pixels")
# plt.xlim(0, 1.0)
# plt.show()

# to actually visualize the effect of `CHAIN_APPROX_SIMPLE`, we need a proper image
# image1 = image.copy()
# img_gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
 
# ret, thresh1 = cv2.threshold(img_gray1, 150, 255, cv2.THRESH_BINARY)
# contours2, hierarchy2 = cv2.findContours(thresh1, cv2.RETR_TREE,
#                                                cv2.CHAIN_APPROX_SIMPLE)
# image_copy2 = image1.copy()
# cv2.drawContours(image_copy2, contours2, -1, (0, 255, 0), 2, cv2.LINE_AA)
# cv2.imshow('SIMPLE Approximation contours', image_copy2)
# # cv2.waitKey(0)
# image_copy3 = image1.copy()
# for i, contour in enumerate(contours2): # loop over one contour area
#    for j, contour_point in enumerate(contour): # loop over the points
#        # draw a circle on the current contour coordinate
#        cv2.circle(image_copy3, ((contour_point[0][0], contour_point[0][1])), 2, (0, 255, 0), 2, cv2.LINE_AA)
# # see the results
# cv2.imshow('CHAIN_APPROX_SIMPLE Point only', image_copy3)
# # cv2.waitKey(0)
# # cv2.imwrite('contour_point_simple.jpg', image_copy3)
# # cv2.destroyAllWindows()









# cv2.imshow("mask", mask)
cv2.imshow("image", image)

# Wait for a key press and exit when the 'Esc' key is pressed
key = cv2.waitKey(0)
if key == 27:
    cv2.destroyAllWindows()




# import cv2
# import numpy as np
# # video = cv2.VideoCapture("2023-11-08 16-37-00.mkv")
# video = cv2.imread("ice_image.jpg")


# while True:
#     ret, orig_frame = video.read()
#     if not ret:
#         video = cv2.VideoCapture("road_car_view.mp4")
#         continue
#     frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     low_yellow = np.array([18, 94, 140])
#     up_yellow = np.array([48, 255, 255])
    
#     mask = cv2.inRange(hsv, low_yellow, up_yellow)
#     edges = cv2.Canny(mask, 75, 150)
#     lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
#     if lines is not None:
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
#     cv2.imshow("frame", frame)
#     cv2.imshow("edges", edges)
#     key = cv2.waitKey(1)
#     if key == 27:
#         break
# video.release()
# cv2.destroyAllWindows()
