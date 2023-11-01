import os
import cv2

def set_logitech_c920_focus(focus_value):
    os.system(f"guvcview -f -m {focus_value}")

def capture_and_display_webcam():
    cap = cv2.VideoCapture(0)  # Open the default camera (usually the webcam)

    if not cap.isOpened():
        print("Failed to open the webcam.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        cv2.imshow("Before Focus Adjustment", frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("B")


# if __name__ == "__main":
focus_value_before = 0  # Initial focus value
focus_value_after = 200  # Adjust this value as needed

print("Capturing webcam view before focus adjustment...")
# capture_and_display_webcam()

print("Adjusting focus...")
set_logitech_c920_focus(focus_value_after)

print("Capturing webcam view after focus adjustment...")
capture_and_display_webcam()
print("C")
