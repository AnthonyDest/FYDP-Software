import cv2
import os
import time
import subprocess

class VideoRecorder:
    def __init__(self):
        self.video_writer = None

    def record_frame(self, frame):
        if self.video_writer is None:
            current_time = time.strftime("%Y%m%d-%H%M%S")
            output_dir = "/home/fydp/Documents/FYDP-Software/videos"
            os.makedirs(output_dir, exist_ok=True)
            self.output_file = os.path.join(output_dir, f"output_{current_time}.avi")
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            height, width, _ = frame.shape
            self.video_writer = cv2.VideoWriter(self.output_file, fourcc, 30, (width, height), isColor=True)

        # Write frame to video
        self.video_writer.write(frame)

    def stop_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
            print("Video saved.")
            return self.output_file  # Returning the path of the saved video

def main():
    # Initialize video recorder
    video_recorder = VideoRecorder()

    # Open webcam
    cap = cv2.VideoCapture(0)

    try:
        print("Recording started...")

        # Record for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5:  # Record for 5 seconds
            ret, frame = cap.read()
            if not ret:
                break

            # Record frame
            video_recorder.record_frame(frame)

    finally:
        # Release webcam and stop recording
        cap.release()
        saved_video_path = video_recorder.stop_recording()

        # Open the directory containing the saved video file
        open_directory_command = f"xdg-open {os.path.dirname(saved_video_path)}"
        subprocess.Popen(open_directory_command, shell=True)

if __name__ == "__main__":
    main()
