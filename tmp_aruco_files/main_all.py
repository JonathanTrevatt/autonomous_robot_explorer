import cv2
import sys
import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

#import rawpy
#import imageio
#import vispy
#import vispy.app
#from vispy.util.filter import gaussian_filter
#import vispy.scene
#from vispy.scene import visuals
#from vispy.visuals import transforms

# ---------------------------------
def rgbplot(img_rgb, angle1=45, angle2=45):
    r, g, b = cv2.split(img_rgb)
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    pixel_colors = img_rgb.reshape((np.shape(img_rgb)[0] * np.shape(img_rgb)[1], 3))
    norm = colors.Normalize(vmin=-1.0, vmax=1.0)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    axis.scatter(r.flatten(), g.flatten(), b.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Red")
    axis.set_ylabel("Green")
    axis.set_zlabel("Blue")
    axis.view_init(angle1, angle2)
    plt.show()

def hsvplot(img_rgb, angle1=45, angle2=45):
    hsv_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv_img)
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    pixel_colors = img_rgb.reshape((np.shape(img_rgb)[0] * np.shape(img_rgb)[1], 3))
    norm = colors.Normalize(vmin=-1.0, vmax=1.0)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Hue")
    axis.set_ylabel("Saturation")
    axis.set_zlabel("Value")
    axis.view_init(angle1, angle2)
    plt.show()

def color_swatch_hsv(color1, color2):
    # Normalise to 0 - 1 range for viewing
    lo_square = np.full((8, 10, 3), color1, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), color2, dtype=np.uint8) / 255.0
    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()

def dirtylines(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 150, 200, apertureSize=3)
    plt.imshow(edges)
    plt.show()

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3), (-1, -1))
    dilated = cv2.dilate(edges, element)
    plt.imshow(dilated)
    plt.show()

    minLineLength = 500
    maxLineGap = 20
    lines = cv2.HoughLinesP(dilated, cv2.HOUGH_PROBABILISTIC, np.pi / 180, 150, minLineLength, maxLineGap)
    for x in range(0, len(lines)):
        for x1, y1, x2, y2 in lines[x]:
            pts = np.array([[x1, y1], [x2, y2]], np.int32)
            cv2.polylines(img, [pts], True, (0, 255, 0))
    plt.imshow(img)
    plt.show()

def plot_hsv_vispy(img_hsv_masked, img_rgb_masked):
    # Make a canvas and add simple view
    canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
    view = canvas.central_widget.add_view()

    # create scatter object and fill in the data
    pixel_values = img_hsv_masked.reshape((np.shape(img_hsv_masked)[0] * np.shape(img_hsv_masked)[1], 3))
    pixel_colors = img_rgb_masked.reshape((np.shape(img_rgb_masked)[0] * np.shape(img_rgb_masked)[1], 3))
    norm = colors.Normalize(vmin=-1.0, vmax=1.0)
    norm.autoscale(pixel_colors)
    pixel_colors_norm = norm(pixel_colors).tolist()
    
    view.camera = 'turntable' 
    
    scatter = visuals.Markers()
    scatter.set_data(pixel_values, edge_width=0, face_color=pixel_colors_norm, size=1)
    view.add(scatter)
    
    #axis = visuals.XYZAxis(parent=view.scene) # add a colored 3D axis for orientation
    axis = visuals.Axis(parent=view.scene, pos=[[0,0], [255,0]])
    axis.axis_label = "Hue"
    axis.axis_font_size = 10
    view.add(axis)

    axis = visuals.Axis(parent=view.scene, pos=[[255,0], [255,255]])
    view.add(axis)

    axis = visuals.Axis(
        parent=view.scene,
        pos=[[0,0], [0,255]], 
        domain=(0, 255), 
        tick_direction=(-1.0, 0.0), 
        scale_type='linear', 
        axis_color=(1, 0, 0), 
        tick_color=(0.7, 0.7, 0.7), 
        text_color='w', 
        minor_tick_length=5, 
        major_tick_length=20, 
        tick_width=5, 
        tick_label_margin=12, 
        tick_font_size=50, 
        axis_width=5, 
        axis_label="Label", 
        axis_label_margin=35, 
        axis_font_size=50, 
        font_size=50, 
        anchors=None)
    view.add(axis)

    axis = visuals.Axis(parent=view.scene, pos=[[0,255], [255,255]])
    view.add(axis)
    
    cube = visuals.Box(
        width=255, 
        height=255, 
        depth=255, 
        width_segments=5, 
        height_segments=5, 
        depth_segments=5, 
        planes={'+x', '+y', '-z'},
        vertex_colors=None, 
        face_colors=None, 
        color=(0, 0, 0, 0), 
        edge_color=(1, 1, 1, 1))
    view.add(cube)
    cube.transform = transforms.STTransform(translate=(255/2, 255/2, 255/2), scale=(1., 1., 1.))
    input("")

def plot_hsv_mpl(img_hsv_masked, img_rgb_masked):
    h, s, v = cv2.split(img_hsv_masked)
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    pixel_colors = img_rgb_masked.reshape((np.shape(img_rgb_masked)[0] * np.shape(img_rgb_masked)[1], 3))
    norm = colors.Normalize(vmin=-1.0, vmax=1.0)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Hue")
    axis.set_ylabel("Saturation")
    axis.set_zlabel("Value")
    plt.show()

def vsplit(img, vdiv):
    # Divide image into vertical slices
    h, w, channels = img.shape
    vsplit = w//5
    part1 = img[:, 0*vsplit:1*vsplit]  
    part2 = img[:, 1*vsplit:2*vsplit]  
    part3 = img[:, 2*vsplit:3*vsplit]  
    part4 = img[:, 3*vsplit:4*vsplit]  
    part5 = img[:, 4*vsplit:5*vsplit]  

    # this is horizontal division
    hsplit = h//1
    top = img[:hsplit, :]
    bottom = img[hsplit:, :]

    # Show divisions
    #cv2.imshow('part1', part1)
    #cv2.imshow('part2', part2)
    #cv2.imshow('part3', part3)
    #cv2.imshow('part4', part4)
    #cv2.imshow('part5', part5)
    #cv2.waitKey(0)
    return (part1, part2, part3, part4, part5)

def get_top_pxl(img):
    for r in range(len(img)):
        for pixel in img[r]:
            if pixel[1] >= 20:
                return r

def get_bottom_pxl(img):
    for r in range(len(img)-1, 0, -1):
        for pixel in img[r]:
            if pixel[1] >= 20:
                return r

def get_pixel_vrange(img):
    return get_bottom_pxl(img) - get_top_pxl(img)

def get_pixel_hrange(img):
    return get_pixel_vrange(cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE))

# ---------------------------------

def hsv_crop(img_hsv):
    hsv_mask = cv2.inRange(img_hsv, (30, 50, 30), (90, 240, 240))  # Segment using inRange() function
    img_hsv_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=hsv_mask)  # Bitwise-AND mask and original image
    img_rgb_masked = cv2.cvtColor(img_hsv_masked, cv2.COLOR_HSV2RGB)
    return img_rgb_masked

def denoise(img):
    kernel = np.ones((10,10),np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel) # Erosion followed by dilation. Removes noise.
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) # Dilation followed by Erosion. Closing small holes inside foreground objects, or small black points on the object.
    return img

def manipulate_tags(img, aruco_tag_size_mm, aruco_tag_size_pxls):
    arucoDict, arucoParams = get_aruco_dict("DICT_4X4_50")
    focal_length, origin, camera_matrix, distCoeffs = init_camera(img)

    #Detect aruco tags
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    print("ids", ids)
    if ids is not None: 
        tags_count = len(ids.flatten())
    else:
        tags_count = 0

    # Draw detected markers
    #cv2.aruco.drawDetectedMarkers(img, corners, ids)
    poses = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_tag_size_mm, camera_matrix, distCoeffs)  
    rvecs, tvecs, _objPoints = poses

    # Determine preference of which tag to warp to (if there is more than one tag)
    warp_tag_prefs = [2, 1, 0]
    warp_tag = None
    for id in warp_tag_prefs:
        if id in ids:
            warp_tag = id
    if warp_tag is None: print("Warp tag not found!")

    # Find index for warp tag
    id_index = None
    for i in range(tags_count):
        if ids[i] == warp_tag:
            id_index = i

    id = ids[id_index]
    (topLeft, topRight, bottomRight,bottomLeft) = corners[id_index][0]
    size = abs(topRight[0] - topLeft[0])/2
    rvec = rvecs[id_index][0] # Rotation with respect to camera
    tvec = tvecs[id_index][0] # Translation with respect to camera

    pts = np.float32(
        [topLeft,
         [topLeft[0] + aruco_tag_size_pxls, topLeft[1]],  # New top right
         [topLeft[0] + aruco_tag_size_pxls, topLeft[1] + aruco_tag_size_pxls],  # New bottom right
         [topLeft[0], topLeft[1] + aruco_tag_size_pxls]  # New bottom left
         ])

    # Warp image
    perspective_transform = cv2.getPerspectiveTransform(corners[id_index][0], pts)
    warped_img = warp_image(img, perspective_transform)
    
    plt.imshow(warped_img)
    plt.show()

    # Detect again on warped image
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    print("Corners", corners)
    (topLeft, topRight, bottomRight,bottomLeft) = corners[id_index][0]
    print(topRight)
    print(warped_img.shape)
    print(int(warped_img.shape[1]))
    cropped_image = warped_img[0:int(topRight[0]+30), 0:int(warped_img.shape[1])]

    return cropped_image

def get_yield_for_height_at_age(height, age, num_shoots):
    yield_per_height_at_age =  (0.00011 * age) + 0.00324
    if age >= 8:
        if age <= 13:
            return yield_per_height_at_age * height * num_shoots

def get_pxl_extent(img, num_splits=5):
    h_pixels_list = []
    splits = vsplit(img, num_splits)
    for split_img in splits:
        h_pixels_list.append(get_pixel_vrange(split_img))
    print(h_pixels_list)
    h_pixels = sum(h_pixels_list) / len(h_pixels_list) # get average pixel height over image
    w_pixels = get_pixel_hrange(img)
    return h_pixels, w_pixels

def init_camera(image):
        # Define camera intrinsic properties (how a camera maps 3D points in the world to 2D points in an image)
        # Matrix can be though of as a rotation matrix concatenated with a translation matrix)
        focal_length = [image.shape[0], image.shape[0]]             # In pixels
        origin = np.array([image.shape[0]/2, image.shape[1]/2])     # principal point (the point that all rays converge) in pixels
        camera_matrix = np.array(
        [[focal_length[0], 0, origin[0]],
        [0, focal_length[1], origin[1]],
        [0, 0, 1]], dtype="double")
        distCoeffs = np.zeros((4, 1))           # lens distortion of camera (None)
        return focal_length, origin, camera_matrix, distCoeffs

def get_aruco_dict(tag_type):
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
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[tag_type])
    arucoParams = cv2.aruco.DetectorParameters_create()
    return arucoDict, arucoParams

def warp_image(img, H):
    '''warp img with homograph H'''
    h,w = img.shape[:2]
    pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
    pts_ = cv2.perspectiveTransform(pts, H)
    xs = []
    ys = []
    for i in range(len(pts_)):
        xs.append(pts_[i][0][0])
        ys.append(pts_[i][0][1])
    xmin, ymin = np.int32(min(xs)), np.int32(min(ys))
    xmax, ymax = np.int32(max(xs)), np.int32(max(ys))

    Ht = np.array([[1,0,-xmin],[0,1,-ymin],[0,0,1]]) # Translate
    result = cv2.warpPerspective(img, Ht.dot(H), ((xmax-xmin), (ymax-ymin)))
    return result

def main():
    # Input parameters
    A_mm2 = 255 * 515     # Total area of the pot
    h_offset = 15           # Distance between the soil and the top of the pot
    age = 10                # Age in days
    image_path = "./images/"
    image_filename = "image3.jpg"

    # inits
    aruco_tag_size_mm = 49
    aruco_tag_size_pxls = 100
    w_mm = 255              # Pot width (mm) (measured)
    d_mm = 515              # Pot depth (mm) (measured)
    h_tray_mm = 70          # Pot height (mm)
    A_mm2 = w_mm * d_mm     # Total area of the pot

    seed_grams_per_mm2 = 55/(255 * 515)  # Grams per mm^2 (55 grams in a standard pot)
    seed_count_per_g = 690/11            # Seed count/gram (Determined experimentally) 
    seed_viability = (420 - 33)/420      # Percentage of seeds that are viable (Determined experimentally)
    num_shoots = A_mm2 * seed_grams_per_mm2 * seed_count_per_g * seed_viability
    num_splits = 5                      # More splits gives more samples for an average
    h_mm_per_pxl = aruco_tag_size_mm/aruco_tag_size_pxls    # Height in mm per height in pixel 

    # Load image (jpg or raw)
    try:    img = rawpy.imread(image_path+image_filename).postprocess()
    except: img = cv2.imread(image_path+image_filename)
    plt.imshow(img)
    plt.show()

    # Warp image to correct for perspective
    warped_img = manipulate_tags(img, aruco_tag_size_mm, aruco_tag_size_pxls)
    plt.imshow(warped_img)
    plt.show()
    #cv2.imshow("Warped", warped_img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    img = warped_img

    # Convert image to HSV colour space
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Perform image cropping/masking to keep only greens
    img = hsv_crop(img_hsv)
    plt.imshow(img)
    plt.show()

    # Remove noise and fill in holes - all pixels left should be part of plant
    img = denoise(img)
    plt.imshow(img)
    plt.show()

    # Measure average maximum height in pixels
    h_pixels, w_pixels = get_pxl_extent(img, num_splits)

    # Convert pixel height to mm
    h_mm = h_mm_per_pxl * h_pixels
    average_max_height = h_mm + h_offset

    average_height = average_max_height/2

    # Get yield estimate based on height (mm), age (days), and estimated shoot count
    yield_estimate = get_yield_for_height_at_age(average_height, age, num_shoots)

    print("num_shoots", num_shoots)
    print("h_pixels", h_pixels)
    print("w_pixels", w_pixels)
    print("h_mm", h_mm)
    print("yield_estimate", yield_estimate)
    print("-----------------------------")


if __name__ == '__main__':
    main()

