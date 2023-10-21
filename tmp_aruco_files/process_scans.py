import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb
import matplotlib.image as mpimg


def rgbplot(img_rgb, angle1=45, angle2=45):
    """
    Displays a 3d RGB plot of an RGB image from a given perspective.
    :param img_rgb:
    :type img_rgb:
    :param angle1:
    :type angle1:
    :param angle2:
    :type angle2:
    :return:
    :rtype:
    """
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


def rgb_plot_all(img_rgb):
    """
    Displays a 3d RGB plot of an RGB image from principal perspectives (RG, RB, GB).
    :param img_rgb:
    :type img_rgb:
    :return:
    :rtype:
    """
    rgbplot(img)
    print("RGB plot complete")
    rgbplot(img, 0, 0)
    print("RGB plot complete")
    rgbplot(img, 90, 0)
    print("RGB plot complete")
    rgbplot(img, 0, 90)
    print("RGB plot complete")


def hsv_plot_all(img_rgb):
    """
    Displays a 3d HSV plot of an RGB image from principal perspectives (HS, HV, SV).
    :param img_rgb:
    :type img_rgb:
    :return:
    :rtype:
    """
    hsvplot(img)
    print("HSV plot complete")
    hsvplot(img, 0, 0)
    print("HSV plot complete")
    hsvplot(img, 90, 0)
    print("HSV plot complete")
    hsvplot(img, 0, 90)
    print("HS plot complete")


def hsvplot(img_rgb, angle1=45, angle2=45):
    """
    Displays a 3d HSV plot of an RGB image from a given perspective.
    :param img_rgb:
    :type img_rgb:
    :param angle1:
    :type angle1:
    :param angle2:
    :type angle2:
    :return:
    :rtype:
    """
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
    """
    Visualize two HSV defined colors side by side for comparison.
    :param color1:
    :type color1:
    :param color2:
    :type color2:
    :return:
    :rtype:
    """
    # Normalise to 0 - 1 range for viewing
    lo_square = np.full((8, 10, 3), color1, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), color2, dtype=np.uint8) / 255.0
    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()


def imshow_windowed(image):
    """
    Opens a canvas in a new window to display an image at full resolution.
    :param image:
    :type image:
    :return:
    :rtype:
    """
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)  # Create window with freedom of dimensions
    # cv2.resizeWindow("output", 400, 300)              # Resize window to specified dimensions
    cv2.imshow("output", image)  # Show image
    cv2.waitKey(0)


def hsv_mask(rgb_img, hsv_color1, hsv_color2):
    """
    Removes pixels that are not between the specified hsv colors.
    :param rgb_img:
    :type rgb_img:
    :param hsv_color1:
    :type hsv_color1:
    :param hsv_color2:
    :type hsv_color2:
    :return:
    :rtype:
    """
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_img, hsv_color1, hsv_color2)  # Segment using inRange() function
    result = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)  # Bitwise-AND mask and original image
    rgb_img = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    return rgb_img

def locate_color(image, rgb_set):
    """
    Returns a list of pixel locations where the pixel in the image is a specified color.
    :param image: 
    :type image: 
    :param rgb_set: 
    :type rgb_set: 
    :return: 
    :rtype: 
    """
    locations = list()
    for j in range(0, image.shape[0]):
        for i in range(0, image.shape[1]):
            if image[j][i] == rgb_set:
                locations.append([j, i])
    return locations


from numba import njit


@njit
def threshold_njit(T, image):
    # grab the image dimensions
    h = image.shape[0]
    w = image.shape[1]

    # loop over the image, pixel by pixel
    for y in range(0, h):
        for x in range(0, w):
            # threshold the pixel
            image[y, x] = 255 if image[y, x] >= T else 0

    # return the thresholded image
    return image


@njit
def get_unique_colors(image):
    # grab the image dimensions
    h = image.shape[0]
    w = image.shape[1]

    # loop over the image, pixel by pixel
    for y in range(0, h):
        for x in range(0, w):
            # threshold the pixel
            image[y, x] = 255 if image[y, x] >= T else 0

    # return the thresholded image
    return image


file = "./images/scans/A.png"  # 6960x4952
img = cv2.imread(file)  # Read image
img = cv2.resize(img, (1405, 1000))
plt.rcParams['figure.figsize'] = (5, 5)
img = cv2.fastNlMeansDenoisingColored(img, h=3, hColor=3, templateWindowSize=7, searchWindowSize=21)

plt.imshow(img)
plt.show()
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# cv2.imwrite('./images/scans/A_denoised.png', img)
hsv_plot_all(img)

img = hsv_mask(img, np.array([12, 10, 0]), np.array([75, 250, 150]))
plt.imshow(img)
plt.show()

hsv_plot_all(img)
