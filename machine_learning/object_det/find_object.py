# Identify contours within image, and classify individual contours based on LBP texture analysis
# Implementation: python3 find_object.py -i ~/gitrepo/image_proc/mache5.JPEG
# Import the necessary packages
from texture_class import LocalBinaryPatterns
import argparse
import cv2
import imutils
import numpy as np
import joblib

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=False,
	help="path to input image")
args = vars(ap.parse_args())

# Define image dimensions to use when resizing
WIDTH = 600
HEIGHT = 480

if not args.get("image", False):

    # If no user-supplied path, load the a preset image
    original = cv2.imread("/home/pyimagesearch/software/python/camera_test/still_image.jpg")

else:
    # Otherwise, use the user-supplied path
    original = cv2.imread(args["image"])

# Resize image 
dim = (WIDTH, HEIGHT)
resized = cv2.resize(original, dim)
result = resized.copy()

# Define HSV boundaries for desired color
blueLower = (50, 80, 10)
blueUpper = (100, 255, 255)

greenLower = (40, 80, 10)
greenUpper = (80, 255, 255)

# Define RGB red color for drawing
red = (0, 0, 255)

# Initialize the local binary pattersn descriptor
desc = LocalBinaryPatterns(24, 8)

# Apply Gaussian Blur and convert to HSV colorspace
blurred = cv2.GaussianBlur(result, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

# Apply color mask, with erosion-dilation sequence to eliminate tiny blotches
masked = cv2.inRange(hsv, greenLower, greenUpper)
masked = cv2.erode(masked, None, iterations=2)
masked = cv2.dilate(masked, None, iterations=6)

# Find contours in the masked image 
contours = cv2.findContours(masked.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Parse the contours list to account for varying OpenCV handling methods
contours = imutils.grab_contours(contours)

# Initialize array for storing contour images, centroids, and 
# classification predictions
cropped_images = []
centroids = []
classifications = []

# Only proceed if at least one countour was found
if len(contours) > 0:
    
	for i in range(len(contours)):
		# Create black canvas matching dimensions of the image
		mask = np.zeros(resized.shape[:2], dtype = "uint8")
		# Fill in the detected contour in white against black background
		cv2.drawContours(mask, contours, i, color=(255,255,255),thickness = -1)
		# Perform bitwise AND to extract image regions within contours
		bitwiseAnd = cv2.bitwise_and(resized, resized, mask = mask)
		# Generate and draw a bounding box for the detected contour to be displayed on final result
		x,y,w,h = cv2.boundingRect(contours[i])
		
		# Skip bounding boxes measuring less than 20 x 20 pixels
		if w and h < 20:
			continue
		
		# Otherwise draw a bounding box around the contour and save it
		# as a separate image			
		else:
			cv2.rectangle(result, (x,y), (x+w, y+h), red, 2)
			# Store the cropped contour image and display
			# cropped_images.append(bitwiseAnd[y:y+h, x:x+w])
			cropped_images.append(bitwiseAnd)
			
			# Calculate bounding box centroid, adjust to be measured 
			# from the center of the camera frame, and store coordinates
			# Positive x-values are to the right of frame centroid
			# Positive y-values are above frame centroid
			mid = ((x + w//2) - WIDTH//2, HEIGHT//2 - (y + h//2))
			# mid = (x + w//2, y + h//2)
			centroids.append(mid)

else:
    print("No contours found!")
    
# Apply texture analysis to contour-bound cropped images
# First load the pre-trained texture model
loaded_model = joblib.load('trained_texturemodel.sav')
print("[STATUS] Loading trained model...")
# loop over the extracted image segments and classify according to texture
i = 0
for image in cropped_images:
	# load the image, convert it to grayscale, describe it,
	# classify it, and log the classification outcome
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	hist = desc.describe(gray)
	prediction = loaded_model.predict(hist.reshape(1, -1))
	classifications.append(prediction[0])
	# display the image and the prediction
	# cv2.putText(image, prediction[0], (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
		# 1.0, (0, 0, 255), 3)
	# cv2.imshow("Image", image)
	# cv2.waitKey(0)

segmented = cv2.bitwise_and(resized, resized, mask = masked)

# Display and print final results
cv2.imshow("Contours", result)
cv2.imshow("Color masked", masked)
#cv2.imshow("Segmented Image", segmented)
#cv2.imshow("Original Resized", resized)
print("Number of plants found: {}".format(len(contours)))
print("Plant locations: {}".format(centroids))
print("Classifications: {}".format(classifications))	
# cv2.imwrite("detected_contours.jpeg", result)
# cv2.imwrite("mask.jpeg", masked)
cv2.imwrite("segmented_green.jpeg", segmented)
cv2.waitKey(0)

# # Display the contour-bound image segments
# for image in cropped_images:
	# cv2.imshow("Cropped", image)
	# cv2.waitKey(0)	
	
cv2.destroyAllWindows()
