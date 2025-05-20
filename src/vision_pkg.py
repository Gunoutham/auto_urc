import cv2
import cv2.aruco as aruco
import numpy as np


# def detectAruco(frame):
#     aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
#     aruco_params = aruco.DetectorParameters()
#     gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, aruco_params)

#     if ids is not None:
#         return ids[0][0]
#     return None

def detectAruco(frame):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters_create()

    aruco_params.adaptiveThreshWinSizeMin = 3
    aruco_params.adaptiveThreshWinSizeMax = 23
    aruco_params.adaptiveThreshConstant = 7
    aruco_params.minMarkerPerimeterRate = 0.03
    aruco_params.maxMarkerPerimeterRate = 4.0
    aruco_params.polygonalApproxAccuracyRate = 0.03
    aruco_params.minCornerDistanceRate = 0.05
    aruco_params.minDistanceToBorder = 3
    aruco_params.filterQuads = True

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

    # If camera calibration data is available, use it to undistort the image
    # This step can improve accuracy by reducing lens distortion
    # Uncomment and update the following line if you have the camera matrix and distortion coefficients
    # frame = cv2.undistort(gray_img, camera_matrix, dist_coeffs)

    # Detect the markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=aruco_params)

    if ids is not None:
        valid_ids = [ids[i][0] for i in range(len(ids)) if ids[i] is not None]
        if valid_ids:
            return valid_ids[0]

    return None

def detectMallet(frame):
    '''Detect Mallet'''
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([25, 255, 255])

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest orange contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 500:
            # Get bounding box around largest contour
            x, y, w, h = cv2.boundingRect(largest)
            # Calculate center of bounding box
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            return (cx, cy, area)

    return None


def detectBottle(frame):
    '''Detect Bottle'''

def to_obj(frame):
    '''Position the object to the center of the frame and move towards it'''