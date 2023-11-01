import cv2

cam = cv2.VideoCapture(1)
focus_value = 50
cam.set(cv2.CAP_PROP_FOCUS, focus_value)

while True:
    ret, image = cam.read()
    cv2.imshow('Image', image)
    k = cv2.waitKey(1)
    if k != -1:
        break

cam.release()
cv2.destroyAllWindows()
