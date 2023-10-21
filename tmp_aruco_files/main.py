import cv2
import matplotlib.pyplot as plt
import numpy as np
import rawpy

def vsplit(img):
    """Divides an image into 5 sub-images, sliced vertically.

    Args:
        img (array): Input image.

    Returns:
        array: 5 sub-images, sliced vertically.
    """
    # Divide image into vertical slices
    h, w, channels = img.shape
    vsplit = w//5
    part1 = img[:, 0*vsplit:1*vsplit]  
    part2 = img[:, 1*vsplit:2*vsplit]  
    part3 = img[:, 2*vsplit:3*vsplit]  
    part4 = img[:, 3*vsplit:4*vsplit]  
    part5 = img[:, 4*vsplit:5*vsplit]  

    return (part1, part2, part3, part4, part5)

def get_top_pxl(img):
    """Returns the location of the top-most non-zero pixel in an image.

    Args:
        img (array): Input image.

    Returns:
        int: The location of the top-most non-zero pixel in an image.
    """
    for r in range(len(img)):
        for pixel in img[r]:
            if pixel[1] >= 20:
                return r

def get_bottom_pxl(img):
    """Returns the location of the bottom-most non-zero pixel in an image.

    Args:
        img (array): Input image.

    Returns:
        int: The location of the bottom-most non-zero pixel in an image.
    """
    for r in range(len(img)-1, 0, -1):
        for pixel in img[r]:
            if pixel[1] >= 20:
                return r

def get_pixel_vrange(img):
    """Returns the maximum range of non-zero pixels in the vertical direction.

    Args:
        img (array): Input image.

    Returns:
        int: Maximum range of non-zero pixels in the vertical direction.
    """
    return get_bottom_pxl(img) - get_top_pxl(img)

def get_pixel_hrange(img):
    """Returns the maximum range of non-zero pixels in the horizontal direction.

    Args:
        img (array): Input image.

    Returns:
        int: Maximum range of non-zero pixels in the horizontal direction.
    """
    return get_pixel_vrange(cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE))

def hsv_crop(img_hsv, threshold_min = (30, 50, 30), threshold_max = (90, 240, 240)):
    """Segment a given (hsv format) image based on thresholds in the hsv colour space.

    Args:
        img_hsv (array): Input image to be segmented (in hsv format).
        threshold_min (tuple, optional): The minimum values for the 
            threshold of pixel colours to keep. Defaults to (30, 50, 30).
        threshold_max (tuple, optional): The maximum values for the 
            threshold of pixel colours to keep. Defaults to (90, 240, 240).

    Returns:
        array: Input image with all pixels outside of selected hsv range removed.
    """
    # Segment using inRange() function
    hsv_mask = cv2.inRange(img_hsv, threshold_min, threshold_max)  
    # Bitwise-AND mask and original image
    img_hsv_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=hsv_mask)  
    img_rgb_masked = cv2.cvtColor(img_hsv_masked, cv2.COLOR_HSV2RGB)
    return img_rgb_masked

def denoise(img, kernal_size = 10):
    """Applies opening and closing operations using the given square kernal size. 
    Removes noise from the image by deleting features that are smaller than the kernal size.

    Args:
        img (array): Image to be denoised.
        kernal_size (int, optional): Size of the square kernal used for the opening 
            and closing operations. Defaults to 10.

    Returns:
        array: Input image after denoising.
    """
    kernel = np.ones((kernal_size, kernal_size),np.uint8)
    # Erosion followed by dilation. Removes noise.               
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    # Dilation followed by Erosion. Closing small holes inside foreground objects, 
    # or small black points on the object. 
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
    return img

def manipulate_tags(img, aruco_tag_size_pxls):
    """Corrects perspective of an image so that the camera plane is parallel to, 
    and with the same orientation as the surface and corners of an aruco tag. 
    Then, crops the image to remove all pixels below the top of the aruco tag.

    Args:
        img (array): An image that includes a 4x4_50 aruco tag.
        aruco_tag_size_pxls (int): The desired size of the aruco tag (in pixels) 
            for the output image. 

    Returns:
        array: Warped and cropped image.
    """
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    focal_length, origin, camera_matrix, distCoeffs = init_camera(img)

    # Detect aruco tags
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    print("ids", ids)
    if ids is not None: 
        tags_count = len(ids.flatten())
    else:
        tags_count = 0

    # Determine preference of which tag to warp to (if there is more than one tag detected)
    warp_tag_prefs = [2, 1, 0]
    warp_tag = None
    for id in warp_tag_prefs:
        if id in ids:
            warp_tag = id
    if warp_tag is None: print("Warp tag not found!")

    # Find index for tag to use for image warping
    id_index = None
    for i in range(tags_count):
        if ids[i] == warp_tag:
            id_index = i

    # For the aruco tag, note the current locations of each of the corners
    id = ids[id_index]
    (topLeft, topRight, bottomRight,bottomLeft) = corners[id_index][0]
    
    # Compute the desired new corner locations after the image is warped
    pts = np.float32(
        [topLeft,
         [topLeft[0] + aruco_tag_size_pxls, topLeft[1]],  # New top right
         # New bottom right
         [topLeft[0] + aruco_tag_size_pxls, topLeft[1] + aruco_tag_size_pxls],  
         [topLeft[0], topLeft[1] + aruco_tag_size_pxls]  # New bottom left
         ])

    # Warp the image to transform corners to new positions
    perspective_transform = cv2.getPerspectiveTransform(corners[id_index][0], pts)
    warped_img = warp_image(img, perspective_transform)
    warped_img = cv2.rotate(warped_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    plt.imshow(warped_img)
    plt.show()

    # Run tag detection again on warped image
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, 
        parameters=arucoParams)
    print("Corners", corners)
    (topLeft, topRight, bottomRight,bottomLeft) = corners[id_index][0]
    print(topRight)
    print(warped_img.shape)
    print(int(warped_img.shape[1]))

    

    # Crop image below the tag to reduce inaccuracy due to plants overhanging the tray 
    #cropped_image = warped_img[0:int(topRight[0]+30), 0:int(warped_img.shape[1])]
    cropped_image = warped_img[0:int(topLeft[0]+30), 0:int(warped_img.shape[1])]
    return cropped_image

def get_yield_for_height_at_age(height, age, num_shoots):
    """Uses a known relationship between the height and age of microgreens, 
        and the corresponding yield to estimate yield.

    Args:
        height (float): An estimate of the average height of microgreens (in mm)
        age (int): The age of the microgreens (in days since planting)
        num_shoots (int): An estimate of the total number of shoots in the 
            population of microgreens

    Returns:
        float: An estimate of the yield of the population of microgreens (in grams)
    """
    yield_per_height_at_age =  (0.00011 * age) + 0.00324
    if age >= 8:
        if age <= 13:
            return yield_per_height_at_age * height * num_shoots

def get_pxl_extent(img, num_splits = 5):
    """Splits the image into multiple sections, and returns the average 
    (across all sections) range in the x and y directions of all non-zero pixels.

    Args:
        img (array): Input image to get extents for.
        num_splits (int, optional): The number of divisions of the image to average across. 
            Defaults to 5.

    Returns:
        float, float: Average horizontal and vertical ranges.
    """
    h_pixels_list = []
    splits = vsplit(img)
    for split_img in splits:
        h_pixels_list.append(get_pixel_vrange(split_img))
    print(h_pixels_list)
    h_pixels = sum(h_pixels_list) / len(h_pixels_list) # get average pixel height over image
    w_pixels = get_pixel_hrange(img)
    return h_pixels, w_pixels

def init_camera(image):
    """Initialise the intrinsic properties of a camera. 
    It is assumed that there is no lens distortion.
    I.e., the focal length, origin, distortion characteristics, and a transformation matrix 
        (rotation and translation) describing how a camera maps 3D points in the world to 
            2D points in an image.

    Args:
        image (array): An image taken by the camera.

    Returns:
        float: (focal_length):  Estimated focal length of the camera.
        array: (origin):            Vector describing the position of the middle pixel 
            in the image.
        array: (camera_matrix):     Transformation matrix describing the rotation and 
            translation of the camera.
        array: (distCoeffs):        Matrix describing the distortion characteristics 
            of the camera (assuming no distortion).
    """
    # Define camera intrinsic properties (how a camera maps 3D points in the world to 
    #   2D points in an image)
    # Matrix can be though of as a rotation matrix concatenated with a translation matrix)
    focal_length = [image.shape[0], image.shape[0]]             # In pixels
    # principal point (the point that all rays converge) in pixels
    origin = np.array([image.shape[0]/2, image.shape[1]/2])     
    camera_matrix = np.array(
    [[focal_length[0], 0, origin[0]],
    [0, focal_length[1], origin[1]],
    [0, 0, 1]], dtype="double")
    distCoeffs = np.zeros((4, 1))           # lens distortion of camera (None)
    return focal_length, origin, camera_matrix, distCoeffs

def warp_image(img, H):
    """Warps image by transformation matrix (homograph) H.

    Args:
        img (array): Image to warp.
        H (array): Transformation matrix to warp image.

    Returns:
        array: Input image warped by H.
    """
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
    age = 8                # Age in days
    image_path = "./images/"
    image_filename = "T4-front1.jpg"

    # inits
    aruco_tag_size_mm = 49
    aruco_tag_size_pxls = 100
    w_mm = 255              # Pot width (mm) (measured)
    d_mm = 515              # Pot depth (mm) (measured)
    h_tray_mm = 70          # Pot height (mm)
    A_mm2 = w_mm * d_mm     # Total area of the pot

    seed_grams_per_mm2 = 55/(255 * 515)  # Grams per mm^2 (55 grams in a standard pot)
    seed_count_per_g = 690/11            # Seed count/gram (Determined experimentally) 
    seed_viability = (420 - 33)/420      # Viable seeds percentage (Determined experimentally)
    num_shoots = A_mm2 * seed_grams_per_mm2 * seed_count_per_g * seed_viability
    num_splits = 5                      # More splits gives more samples for an average
    h_mm_per_pxl = aruco_tag_size_mm/aruco_tag_size_pxls    # Height in mm per height in pixel 

    # Load image (jpg or raw)
    try:    img = rawpy.imread(image_path+image_filename).postprocess()
    except: img = cv2.imread(image_path+image_filename)
    plt.imshow(img)
    plt.show()

    # Warp image to correct for perspective, and cut unwanted parts
    warped_img = manipulate_tags(img, aruco_tag_size_pxls)    
    plt.imshow(warped_img)
    plt.show()

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

