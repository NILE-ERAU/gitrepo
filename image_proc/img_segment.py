# Import the necessary packages
import argparse
import cv2

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=False,
	help="path to input image")
args = vars(ap.parse_args())

if not args.get("image", False):

    # If no user-supplied path, load the a preset image
    image = cv2.imread("/home/pyimagesearch/software/python/camera_test/still_image.jpg")

else:
    # Otherwise, use the user-supplied path
    image = cv2.imread(args["image"])


cv2.imshow("Original", image)

# Convert the image to grayscale and implement Gaussian Blur
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11,11), 0)

# instead of manually specifying the threshold value, we can use
# adaptive thresholding to examine neighborhoods of pixels and
# adaptively threshold each neighborhood
thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
cv2.imshow("Mean Adaptive Thresholding", thresh)
cv2.waitKey(0)