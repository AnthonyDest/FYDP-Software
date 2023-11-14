import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from IPython.display import display

# Load the image using cv2
image = cv2.imread('ice_image.jpg', cv2.IMREAD_GRAYSCALE)  # Replace 'your_image.jpg' with the path to your image

# Create a figure with subplots
fig, ax = plt.subplots(1, 2, figsize=(12, 6))
plt.subplots_adjust(bottom=0.35)

# Display the original image
ax[0].imshow(image, cmap='gray')
ax[0].set_title("Original Image")

# Initial Gabor filter parameters
ksize = 9
sigma = 4
theta = np.pi / 4
lambd = 10
gamma = 0.5
psi = 0

# Function to update the Gabor filter and display the result
def update(val):
    global ksize, sigma, theta, lambd, gamma, psi
    
    # Get the parameter values from the sliders
    ksize = int(ksize_slider.val)
    sigma = sigma_slider.val
    theta = theta_slider.val
    lambd = lambd_slider.val
    gamma = gamma_slider.val
    psi = psi_slider.val
    
    # Create the Gabor kernel with updated parameters
    kernel = cv2.getGaborKernel((ksize, ksize), sigma, theta, lambd, gamma, psi, ktype=cv2.CV_32F)

    # Apply the Gabor filter to the image
    filtered_image = cv2.filter2D(image, cv2.CV_8UC3, kernel)
    
    # Update the right subplot with the filtered image
    ax[1].clear()
    ax[1].imshow(filtered_image, cmap='gray')
    ax[1].set_title("Gabor Filtered Image")
    plt.draw()

# Define the slider positions and sizes
ksize_slider = Slider(plt.axes([0.2, 0.2, 0.65, 0.03]), 'ksize', 3, 21, valinit=ksize, valstep=2)
sigma_slider = Slider(plt.axes([0.2, 0.15, 0.65, 0.03]), 'sigma', 0.1, 10.0, valinit=sigma)
theta_slider = Slider(plt.axes([0.2, 0.1, 0.65, 0.03]), 'theta', 0, np.pi, valinit=theta)
lambd_slider = Slider(plt.axes([0.2, 0.05, 0.65, 0.03]), 'lambd', 1, 20, valinit=lambd)
gamma_slider = Slider(plt.axes([0.2, 0.0, 0.65, 0.03]), 'gamma', 0.1, 1.0, valinit=gamma)
psi_slider = Slider(plt.axes([0.2, -0.05, 0.65, 0.03]), 'psi', 0, np.pi, valinit=psi)

# Attach the update function to the slider changes
ksize_slider.on_changed(update)
sigma_slider.on_changed(update)
theta_slider.on_changed(update)
lambd_slider.on_changed(update)
gamma_slider.on_changed(update)
psi_slider.on_changed(update)

# Display the sliders
plt.show()
