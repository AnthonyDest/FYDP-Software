import cv2
import time

# Set the video resolution and framerate
video_resolution = (640, 480)
framerate = 30

# Set the duration of the video recording in seconds
record_duration = 10

# Create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_filename = 'output_video.avi'
out = cv2.VideoWriter(video_filename, fourcc, framerate, video_resolution)

# Open the camera
camera = cv2.VideoCapture(0)

# Set the camera resolution and framerate
camera.set(cv2.CAP_PROP_FRAME_WIDTH, video_resolution[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, video_resolution[1])
camera.set(cv2.CAP_PROP_FPS, framerate)

# Record video
start_time = time.time()
while (time.time() - start_time) < record_duration:
    ret, frame = camera.read()
    if not ret:
        break

    # Write the frame to the output video file
    out.write(frame)

# Release resources
camera.release()
out.release()
cv2.destroyAllWindows()

print(f"Video recorded to {video_filename}")