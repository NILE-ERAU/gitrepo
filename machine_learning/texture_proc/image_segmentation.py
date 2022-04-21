# Import necessary libraries
# Import the necessary packages
from texture_class import LocalBinaryPatterns
import argparse
import cv2
import imutils
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True,
	help="path to the unprocessed images")
	
# Define image dimensions to use when resizing
WIDTH = 600
HEIGHT = 480

# Loop through the images in specified directory	
for imagePath in paths.list_images(args["images"]):
	# load the image, convert it to grayscale, and describe it
	image = cv2.imread(imagePath)
	# Resize image 
	dim = (WIDTH, HEIGHT)
	resized = cv2.resize(image, dim)

	# Define HSV boundaries for desired color
	blueLower = (50, 80, 10)
	blueUpper = (100, 255, 255)

	greenLower = (40, 80, 10)
	greenUpper = (80, 255, 255)
	
	# Apply Gaussian Blur and convert to HSV colorspace
	blurred = cv2.GaussianBlur(result, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# Apply color mask, with erosion-dilation sequence to eliminate tiny blotches
	masked = cv2.inRange(hsv, greenLower, greenUpper)
	masked = cv2.erode(masked, None, iterations=2)
	masked = cv2.dilate(masked, None, iterations=6)

	# Find contours in the masked image 
	contours = cv2.findContours(masked.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	# Perform bitwise AND to extract image regions within contours
	bitwiseAnd = cv2.bitwise_and(resized, resized, mask = mask)
