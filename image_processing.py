import os
import threading
import time
import cv2
import numpy as np
import helper


class pylon_processing:
    def __init__(self):
        self.video_active = False
        self.video_writer = None
        self.frame = None
        self.keep_turning = False
        self.start_turn_time = 0
        pass

    def start_video(self):

        self.cap = cv2.VideoCapture(0)
        ret, frame = self.cap.read()
        self.video_active = True

    # Function to read a still image
    def read_image(self, image_path):
        frame = cv2.imread(image_path)
        return frame

    def record_frame(self, frame):
        if self.video_writer is None:
            print("STARTED RECORDING")
            current_time = time.strftime("%Y%m%d-%H%M%S")
            output_dir = "/home/fydp/Documents/FYDP-Software/videos"
            os.makedirs(output_dir, exist_ok=True)
            self.output_file = os.path.join(output_dir, f"output_{current_time}.avi")
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            height, width, _ = frame.shape
            self.video_writer = cv2.VideoWriter(
                self.output_file, fourcc, 30, (width, height), isColor=True
            )

        # Write frame to video
        self.video_writer.write(frame)

    # Function to capture video from USB webcam
    def capture_video(self):
        print("Video stream loading...")

        # if not self.video_active:
        #     self.video_active = True
        #     self.cap = cv2.VideoCapture(0)  # Use 0 for default webcam
        #     print("Video stream ready.")
        if self.video_active:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: failed to capture image")
            self.frame = frame

            return frame

    # Function to detect orange color
    def detect_orange(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([105, 75, 0])
        upper_orange = np.array([255, 255, 255])
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
            # print("Steer left")
            return -offset
        elif offset > threshold:
            # print("Steer right")
            return offset
        else:
            # print("Center")
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

        # if self.video_active == False:
        # self.video_active == True
        if input_type == "image":
            print("INPUT SOURCE: IMAGE")
            frame = self.read_image(path)
        elif input_type == "video":
            print("INPUT SOURCE: VIDEO")
            self.capture_video()
            # frame = self.frame
            # if frame is None:
            #     print("Error: No frame captured from the video.")
            # return
        # else:
        # ret, frame = self.cap.read()
        if not self.video_active:
            return 0
        frame = self.frame
        orange_position = self.detect_orange(frame)
        if orange_position is not None:
            x, y, w, h = orange_position
            self.draw_box(frame, x, y, w, h)
            distance_from_center = self.compute_distance_from_center(frame, x, w)

            steer_severity = self.steer_severity(
                distance_from_center, frame.shape[1], center_tolerance=10
            )

            # zz TURN IF CLOSE
            print(f"w: {w}")

            if abs(w) > 50 or self.keep_turning:
                if not self.keep_turning:
                    self.start_turn_time = time.time()
                    self.keep_turning = True

                steer_severity *= -1000

                print("TURN LOCK RIGHT")

            if self.keep_turning and abs(w) < 50:
                if time.time() - self.start_turn_time > 5:
                    self.keep_turning = False
                    print("TURN UNLOCK RIGHT")

            self.record_frame(frame)

            # print(f"The pylon is {distance_from_center} pixels away from the center.")
            # print(f"Steer {steer_severity} to get to the center.")

        # if show_frame:
        #     cv2.imshow("Frame", frame)
        #     cv2.waitKey(1)  # Adjust the delay as needed

        # if self.video_writer is None:
        #     current_time = helper.time.strftime("%Y%m%d-%H%M%S")
        #     output_dir = "/home/fydp/Documents/FYDP-Software/videos"
        #     os.makedirs(output_dir, exist_ok=True)
        #     output_file = os.path.join(output_dir, f"output_{current_time}.avi")
        #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #     self.video_writer = cv2.VideoWriter(output_file, fourcc, 30, (1920, 1080), isColor=True)

        # Write frame to video
        # self.video_writer.write(frame)

        return steer_severity

    def stop_and_save_video(self):
        if self.video_writer is not None:
            self.video_writer.release()
            print("Video saved.")

    def record_video(self, resolution=(1920, 1080), framerate=30, display_live=True):
        if self.video_writer is None:
            # Generate output file name with current time
            current_time = helper.time.strftime("%Y%m%d-%H%M%S")
            output_dir = "videos"
            os.makedirs(output_dir, exist_ok=True)
            output_file = os.path.join(output_dir, f"output_{current_time}.avi")

            # Define codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            self.video_writer = cv2.VideoWriter(
                output_file, fourcc, framerate, resolution, isColor=True
            )

            self.display_live = display_live
            # if display_live:
            #     threading.Thread(target=self.display_live_feed, daemon=True).start()

    def stop_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
            print("Video saved.")

    def display_live_feed(self):
        cap = cv2.VideoCapture(0)  # Open default webcam
        while self.display_live:
            ret, frame = cap.read()  # Read frame from webcam
            if ret:
                cv2.imshow("Live Feed", frame)  # Display live feed in a window
                if cv2.waitKey(1) & 0xFF == ord("q"):  # Press 'q' to stop live display
                    break
        cap.release()
        cv2.destroyAllWindows()


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
