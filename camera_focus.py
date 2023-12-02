import cv2

def set_focus(camera, value):
    # Check if the camera supports focus control
    if camera.isOpened():
        if camera.set(cv2.CAP_PROP_AUTOFOCUS, 0.0):  # Disable autofocus
            # Set the focus value (0.0 - 1.0)
            camera.set(cv2.CAP_PROP_FOCUS, value)
            print(f"Focus set to {value}")
        else:
            print("Camera does not support manual focus control.")
    else:
        print("Failed to open the camera.")

def main():
    # Open the webcam
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        print("Failed to open the camera.")
        return

    focus_value = 200  # Adjust this value (0.0 - 1.0) to set the focus
    set_focus(camera, focus_value)

    while True:
        # Capture a frame from the webcam
        ret, frame = camera.read()
        if not ret:
            break

        # Display the frame
        cv2.imshow("Webcam", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close the OpenCV window
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()





# import os
# import cv2

# def set_logitech_c920_focus(focus_value):
#     os.system(f"guvcview -f -m {focus_value}")

# def capture_and_display_webcam():
#     cap = cv2.VideoCapture(0)  # Open the default camera (usually the webcam)

#     if not cap.isOpened():
#         print("Failed to open the webcam.")
#         return

#     while True:
#         ret, frame = cap.read()

#         if not ret:
#             break

#         cv2.imshow("Before Focus Adjustment", frame)
        
#         # Exit on 'q' key press
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()
#     print("B")


# # if __name__ == "__main":
# focus_value_before = 0  # Initial focus value
# focus_value_after = 200  # Adjust this value as needed

# print("Capturing webcam view before focus adjustment...")
# # capture_and_display_webcam()

# print("Adjusting focus...")
# set_logitech_c920_focus(focus_value_after)

# print("Capturing webcam view after focus adjustment...")
# capture_and_display_webcam()
# print("C")
