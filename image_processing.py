import cv2
import numpy as np
import helper


class pylon_processing:
    def __init__(self):
        self.video_active = False
        pass

    # Function to read a still image
    def read_image(self, image_path):
        frame = cv2.imread(image_path)
        return frame

    # Function to capture video from USB webcam
    def capture_video(self):
        print("Video stream loading...")
        self.cap = cv2.VideoCapture(0)  # Use 0 for default webcam
        self.video_active = True
        print("Video stream ready.")
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: failed to capture image")
                break
            yield frame
        self.cap.release()

    # Function to detect orange color
    def detect_orange(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 150, 100])
        upper_orange = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Get the largest contour
            c = max(contours, key=cv2.contourArea)
            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(c)
            return x, y, w, h
        else:
            return None

    # Function to draw bounding box around the pylon
    def draw_box(self, frame, x, y, w, h):
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Function to compute position of the pylon relative to the screen center
    def compute_box_position(self, frame, x, w, threshold=0.2):
        # Get the center x-coordinate of the bounding box
        box_center_x = x + w // 2
        # Get the width of the image
        image_width = frame.shape[1]
        # Compute the threshold region
        threshold_region = image_width * threshold
        # Calculate the offset from the center
        offset = (box_center_x - image_width / 2) / (image_width / 2)
        # Determine the steering value based on the offset
        if offset < -threshold:
            print("Steer left")
            return -offset
        elif offset > threshold:
            print("Steer right")
            return offset
        else:
            print("Center")
            return 0.0

    # Function to compute how far the pylon is from the center
    def compute_distance_from_center(self, frame, x, w):
        """
        Negative value means pylon is to the right
        positive value means pylon is to the left
        """
        # Get the center x-coordinate of the bounding box
        box_center_x = x + w // 2
        # Get the width of the image
        image_width = frame.shape[1]
        # Compute the distance from the center
        distance_from_center = image_width / 2 - box_center_x
        return distance_from_center

    def steer_severity(self, distance_from_center, image_width, center_tolerance=10):
        "Tolerance is in percentage of the image width. Default is 10%."
        # Calculate the maximum distance from center (half of the image width)
        max_distance = image_width / 2

        # Define the threshold for considering the object close to the center
        center_threshold = (center_tolerance / 100) * max_distance

        # If the object is within 10% of the center, return 0 steering
        if abs(distance_from_center) < center_threshold:
            return 0.0

        # # Move the values 5% of max towards center
        # severity = (max_distance - distance_from_center) / max_distance * 0.05
        # severity = distance_from_center
        # zz map distance from center to radians required to steer

        return distance_from_center

    # Function to process image or video and determine pylon position
    def process_pylon(self, input_type="video", path="None", show_frame=True):
        steer_severity = 0

        if input_type not in ["image", "video"]:
            print("Invalid input type. Choose 'image' or 'video'.")
            return

        if self.video_active == False:
            self.video_active == True
            if input_type == "image":
                print("INPUT SOURCE: IMAGE")
                frame = self.read_image(path)
            elif input_type == "video":
                print("INPUT SOURCE: VIDEO")
                frame = next(self.capture_video(), None)
                if frame is None:
                    print("Error: No frame captured from the video.")
                    return
        else:
            ret, frame = self.cap.read()
        orange_position = self.detect_orange(frame)
        if orange_position is not None:
            x, y, w, h = orange_position
            self.draw_box(frame, x, y, w, h)
            distance_from_center = self.compute_distance_from_center(frame, x, w)

            steer_severity = self.steer_severity(
                distance_from_center, frame.shape[1], center_tolerance=10
            )

            print(f"The pylon is {distance_from_center} pixels away from the center.")
            print(f"Steer {steer_severity} to get to the center.")

        if show_frame:
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)  # Adjust the delay as needed

        return steer_severity


class image_processing:
    # initalize all default values
    def __init__(self):
        self.pose_location = None
        self.vision_mask_location = None

    # TODO get pose from april tag
    def update_pose_location(self):
        self.pose_location = helper.Node(0, 0, 0, 0)

    # TODO determines how L/R the robot is from the desired overlap of the water
    # TODO gives location x +=/-=, and y +=/-= of actual position to correct steering
    # zz assumption is that the previous path is always more correct than current sensor input
    def update_vision_mask_location(self):
        self.vision_mask_location = helper.Node(0, 0, 0, 0)
