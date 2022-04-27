# Implementation: python img_split.py -i mache_sprout.jpg

# Import the necessary packages
import argparse
import cv2
import imutils
import numpy as np

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=True,
	help="path to input image")
args = vars(ap.parse_args())

# Load the supply image and split into R-G-B channels
image = cv2.imread(args["image"])
(B, G, R) = cv2.split(image)

zeros = np.zeros(image.shape[:2], dtype = "uint8")
# Display the images
cv2.imshow("Original", image)
cv2.imshow("Blue Channel", cv2.merge([B, zeros, zeros]))
cv2.imshow("Green Channel", cv2.merge([zeros, G, zeros]))
cv2.imshow("Red Channel", cv2.merge([zeros, zeros, R]))

# Waity for a keypress
cv2.waitKey(0)
cv2.destroyAllWindows()
