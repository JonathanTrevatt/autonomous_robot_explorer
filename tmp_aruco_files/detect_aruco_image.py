# import the necessary packages
#import argparse
#import imutils
import cv2
import matplotlib.pyplot as plt
import sys
import numpy as np
import math

# Reading an image
path = "./images/"
filename = "Side_Modified.jpg"
image_path = path + filename
image = cv2.imread(image_path)
type_arg = "DICT_4X4_50"
args = {"image": image_path, "type": type_arg}
if image is None:
    print("Check file path")

# Define names of ArUco tags supported by OpenCV
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# load the input image from disk and resize it
print("[INFO] loading image...")
image = cv2.imread(args["image"])
#image = imutils.resize(image, width=600)





# Verify that the supplied ArUCo tag exists and is supported by OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
    sys.exit(0)
# load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

def float_lists_to_int_tuples(f_lists):
    for i in range(len(f_lists)):
        f_lists[i] = tuple(map(int, f_lists[i]))
    return f_lists

def draw_quad(image, topLeft, topRight, bottomRight, bottomLeft):
    (topLeft, topRight, bottomRight, bottomLeft) = float_lists_to_int_tuples([topLeft, topRight, bottomRight, bottomLeft])

    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
    return image

def draw_quad_centerpoint(image, topLeft, topRight, bottomRight, bottomLeft):
    (topLeft, topRight, bottomRight, bottomLeft) = float_lists_to_int_tuples([topLeft, topRight, bottomRight, bottomLeft])
    # compute and draw the center (x, y)-coordinates of the ArUco marker
    (cX, cY) = get_quad_centerpoint(topLeft, topRight, bottomRight, bottomLeft)
    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
    # draw the ArUco marker ID on the image
    return image

def draw_quad_markerID(image, topLeft, topRight, bottomRight, bottomLeft, markerID):
    (topLeft, topRight, bottomRight, bottomLeft) = float_lists_to_int_tuples([topLeft, topRight, bottomRight, bottomLeft])
    # draw the ArUco marker ID on the image
    cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print("[INFO] ArUco marker ID: {}".format(markerID))
    return image

def get_quad_centerpoint(topLeft, topRight, bottomRight, bottomLeft):
    (topLeft, topRight, bottomRight, bottomLeft) = float_lists_to_int_tuples([topLeft, topRight, bottomRight, bottomLeft])
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    return cX, cY

def warp_quad_to_square(image, topLeft, topRight, bottomRight, bottomLeft):
    (topLeft, topRight, bottomRight, bottomLeft) = float_lists_to_int_tuples([topLeft, topRight, bottomRight, bottomLeft])
    # Coordinates of quadrangle vertices in the source image.
    pts1 = np.float32([topLeft, topRight, bottomRight, bottomLeft])
    # Coordinates of the corresponding quadrangle vertices in the destination image.
    side_length = topRight[0] - topLeft[0]
    pts2 = np.float32(
        [topLeft,
         [topLeft[0] + side_length, topLeft[1]],  # New top right
         [topLeft[0] + side_length, topLeft[1] + side_length],  # New bottom right
         [topLeft[0], topLeft[1] + side_length]  # New bottom left
         ])
    # Apply Perspective Transform Algorithm
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    image = cv2.warpPerspective(image, matrix, (image.shape[0], image.shape[1]))
    return image

def init_camera(image):
    # Define camera intrinsic properties (how a camera maps 3D points in the world to 2D points in an image)
    # Matrix can be though of as a rotation matrix concatenated with a translation matrix)
    focal_length = [image.shape[0], image.shape[0]]             # In pixels
    origin = np.array([image.shape[0]/2, image.shape[1]/2])    # principal point (the point that all rays converge) in pixels
    camera_matrix = np.array(
    [[focal_length[0], 0, origin[0]],
    [0, focal_length[1], origin[1]],
    [0, 0, 1]], dtype="double")
    distCoeffs = np.zeros((4, 1))           # lens distortion of camera (None)
    return focal_length, origin, camera_matrix, distCoeffs

def process_image(image, arucoDict, arucoParams):
    
    focal_length, origin, camera_matrix, distCoeffs = init_camera(image)
    
    #Detect aruco tags
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if ids is None:
        return
    tags_count = len(ids.flatten())

    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    marker_length = 45.00 # mm

    poses = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distCoeffs)  
    rvecs, tvecs, _objPoints = poses
    
    for i in range(tags_count):
        (topLeft, topRight, bottomRight,bottomLeft) = corners[i][0]
        id = ids[i]
        size = abs(topRight[0] - topLeft[0])/2
        rvec = rvecs[i][0] # Rotation with respect to camera
        tvec = tvecs[i][0] # Translation with respect to camera
        print("--------------")
        print("Tag ID:", id)
        print("rvec", rvec)
        print("tvec", tvec)
        rmat, _ = cv2.Rodrigues(rvec)
        pose_mat = cv2.hconcat((rmat, tvec))
        print("Pose matrix:", pose_mat)
        print("--------------")
        if False:
            # Draw the bounding box around the detected ArUCo tag
            image = draw_quad(image, topLeft, topRight, bottomRight, bottomLeft)
            # Draw a point at the center of the detected ArUCo tag
            image = draw_quad_centerpoint(image, topLeft, topRight, bottomRight, bottomLeft)
            # Draw the ID of the detected ArUCo tag
            image = draw_quad_markerID(image, topLeft, topRight, bottomRight, bottomLeft, id)
        # Draw the estimated pose of the tag
        cv2.drawFrameAxes(image, camera_matrix, distCoeffs, rvec, tvec, size, thickness = 3)
    
        # Choose the first tag detected for warping function
        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
        image = warp_quad_to_square(image, topLeft, topRight, bottomRight, bottomLeft)

    return image

image = process_image(image, arucoDict, arucoParams)
#save matrix/array as image file
isWritten = cv2.imwrite('images\image-2.png', image)
if isWritten:
	print('Image is successfully saved as file.')

# Show image
window_name = 'image'  # Name display window for image
cv2.imshow("Image", image)  # Show the image using cv2.imshow() method
# wait for user key press (necessary to avoid Python kernel form crashing)
cv2.waitKey(0)
cv2.destroyAllWindows()  # closing all open windows (after key press)
