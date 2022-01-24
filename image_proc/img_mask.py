# Import necessary libraries
import cv2
import argparse

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
    image = cv2.imread("/home/pyimagesearch/software/python/camera_test/still_image.jpg")

else:
    # Otherwise, use the user-supplied path
    image = cv2.imread(args["image"])

# Resize the image for display purposes


# Display the original image
#cv2.imshow("Original", image)
#print("Image width: {}, Image height: {}".format(image.shape[1], image.shape[0]))

# Resize image 
dim = (WIDTH, HEIGHT)
resized = cv2.resize(image, dim)
cv2.imshow("Resized Original", resized)

# Define HSV boundaries for desired color
blueLower = (50, 80, 10)
blueUpper = (100, 255, 255)

greenLower = (30, 80, 10)
greenUpper = (80, 255, 255)

# Apply Gaussian Blur and convert to HSV colorspace
blurred = cv2.GaussianBlur(resized, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

# Apply color mask, with erosion-dilation sequence to eliminate tiny blotches
masked = cv2.inRange(hsv, greenLower, greenUpper)
masked = cv2.erode(masked, None, iterations=2)
masked = cv2.dilate(masked, None, iterations=2)

# Combine mask with original image to isolate desired color image, save result
bitwiseAnd = cv2.bitwise_and(resized, resized, mask = masked)
cv2.imwrite("mache6.jpeg", bitwiseAnd)
##cv2.imshow("HSV Colorspace", hsv)
cv2.imshow("Mask", masked)
cv2.imshow("Color Masked", bitwiseAnd)
cv2.waitKey(0)



                           



