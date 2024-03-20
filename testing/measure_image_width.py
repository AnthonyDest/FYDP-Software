import cv2
import sys

sys.path.append("../FYDP-Software")
import image_processing

pylon_camera = image_processing.pylon_processing()

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    cv2.imshow("Original", frame)

    orange_position = pylon_camera.detect_orange(frame)
    if orange_position is not None:
        x, y, w, h = orange_position
        pylon_camera.draw_box(frame, x, y, w, h)

        print(f"w: {w}")

    cv2.imshow("Modified", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
