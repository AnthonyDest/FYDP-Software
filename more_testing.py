import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image using cv2

image = cv2.imread('ice_image.jpg')  # Replace 'your_image.jpg' with the path to your image

# Create a list of masks
masks = [
    ('Original', image),
    ('Gaussian Blur', cv2.GaussianBlur(image, (15, 15), 0)),
    ('Sobel X', cv2.Sobel(image, cv2.CV_8U, 1, 0, ksize=5)),
    ('Sobel Y', cv2.Sobel(image, cv2.CV_8U, 0, 1, ksize=5)),
    ('Median Filter', cv2.medianBlur(image, 5)),
]

# Create a custom sharpening kernel
sharpening_kernel = np.array([[-1, -1, -1],
                              [-1, 9, -1],
                              [-1, -1, -1]])
custom_kernel_mask = cv2.filter2D(image, -1, sharpening_kernel)

# Create erosion and dilation masks
kernel = np.ones((5, 5), np.uint8)
erosion_mask = cv2.erode(image, kernel, iterations=1)
dilation_mask = cv2.dilate(image, kernel, iterations=1)

# Display images with masks applied
fig, axes = plt.subplots(2, 3, figsize=(12, 8))
axes = axes.ravel()

for i, (mask_name, mask) in enumerate(masks):
    axes[i].imshow(cv2.cvtColor(mask, cv2.COLOR_BGR2RGB))
    axes[i].set_title(mask_name)

axes[-3].imshow(custom_kernel_mask, cmap='gray')
axes[-3].set_title('Custom Kernel')

axes[-2].imshow(erosion_mask, cmap='gray')
axes[-2].set_title('Erosion')

axes[-1].imshow(dilation_mask, cmap='gray')
axes[-1].set_title('Dilation')

plt.tight_layout()
plt.show()
