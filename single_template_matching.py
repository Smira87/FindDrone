# import the necessary packages
import argparse
import cv2
from math import asin, atan2, cos, degrees, radians, sin, pi
import time

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=True,
	help="path to input image where we'll apply template matching")
ap.add_argument("-t", "--template", type=str, required=True,
	help="path to template image")
args = vars(ap.parse_args())

# load the input image and template image from disk, then display
# them on our screen
print("[INFO] loading images...")
image = cv2.imread(args["image"])
template = cv2.imread(args["template"])
cv2.imshow("Image", image)
cv2.imshow("Template", template)

# convert both the image and template to grayscale
imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

# perform template matching
print("[INFO] performing template matching...")
result = cv2.matchTemplate(imageGray, templateGray,
	cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)

# determine the starting and ending (x, y)-coordinates of the
# bounding box
(startX, startY) = maxLoc
endX = startX + template.shape[1]
endY = startY + template.shape[0]
# draw the bounding box on the image
TEMPLATE_CENTER_X = startX + template.shape[1]/2
TEMPLATE_CENTER_Y = startY + template.shape[0]/2
cv2.rectangle(image, (startX, startY), (endX, endY), (255, 0, 0), 3)
# find coords of center of main image
IMAGE_CENTER_X = image.shape[1]/2
IMAGE_CENTER_Y = image.shape[0]/2
# draw dot in the center of an image
cv2.circle(image, (int(IMAGE_CENTER_X), int(IMAGE_CENTER_Y)), 3, (0, 0, 255), 5)
# show the output image
cv2.imshow("Output", image)
cv2.waitKey(0)

# https://www.google.com.ua/maps/place/50%C2%B036'02.3%22N+30%C2%B039'05.3%22E/@50.6006414,30.6488891,17z/data=!3m1!4b1!4m4!3m3!8m2!3d50.600638!4d30.651464?hl=ru&entry=ttu
TEMPLATE_CENTER_LAT = 50.600638
TEMPLATE_CENTER_LON = 30.651464
AZ = 335

def get_distance_and_angle(x1, y1, x2, y2):
    delta_x = (x1 - x2)
    delta_y = (y1 - y2)
    distance_in_pixels = (delta_y**2 + delta_x**2)**0.5
    distance = distance_in_pixels * 0.00038

    angle_radian = atan2(delta_y, delta_x)
    angle_degrees = 90 - angle_radian*(180/pi)
    return (distance, angle_degrees)

def get_point_at_distance(lat1, lon1, d, bearing, R=6371):
    """
    lat: initial latitude, in degrees
    lon: initial longitude, in degrees
    d: target distance from initial
    bearing: (true) heading in degrees
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    a = radians(bearing)
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1) * sin(d/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(d/R) * cos(lat1),
        cos(d/R) - sin(lat1) * sin(lat2)
    )
    return (degrees(lat2), degrees(lon2),)

distance, angle_degrees = get_distance_and_angle(TEMPLATE_CENTER_X, TEMPLATE_CENTER_Y, IMAGE_CENTER_X, IMAGE_CENTER_Y)

real_angle = AZ - angle_degrees

drone_coords = get_point_at_distance(TEMPLATE_CENTER_LAT, TEMPLATE_CENTER_LON, distance, real_angle)
print(drone_coords)