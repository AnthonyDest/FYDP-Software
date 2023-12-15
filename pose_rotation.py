import cv2
import numpy as np

cap = cv2.VideoCapture(0)
tag_family = 'tag36h11'
detector_params = cv2.aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)
    
    if ids is not None:
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 14.0, fx=1000.0, fy=1000.0, cx=frame.shape[1] / 2, cy=frame.shape[0] / 2)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.aruco.drawAxis(frame, np.array([100, 100, 0]), rvec, tvec, 100)
            
            euler_angles = cv2.Rodrigues(rvec)[0]
            
            print(f"Tag ID: {ids[i]}")
            print(f"Translation (XYZ): {tvec[0]}")
            print(f"Rotation (Euler XYZ): {euler_angles}")
    
    cv2.imshow("AprilTag Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
