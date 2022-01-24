# Import the necessary libraries
import cv2
import argparse
import numpy as np
##import glob

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=False,
	help="path to input image")
args = vars(ap.parse_args())

if not args.get("image", False):

    # If no user-supplied path, load a preset image
    image = cv2.imread("/home/pyimagesearch/software/python/camera_test/still_image.jpg")

else:
    # Otherwise, use the user-supplied path
    image = cv2.imread(args["image"])
    
# Define the automatic canny-edged detection function
def auto_canny(image, sigma=0.33):
    # Compute the median of the single-channel pixel intensities
    v = np.median(image)
    
    # Apply the automatic Canny edge detection using the compund median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    
    # Return the processed image
    return edged

# Convert the original image to grayscale and apply aslight  gaussian blur
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)

# Apply both manual and automatic canny edge detection schemes for comparison
wide = cv2.Canny(blurred, 10, 200)
tight = cv2.Canny(blurred, 255, 250)
auto = auto_canny(blurred)

# Display the results
cv2.imshow("Original", image)
cv2.imshow("Tight Canny Detection", tight)
cv2.imshow("Wide Canny Detection", wide)
cv2.imshow("Automatic Canny", auto)

cv2.imwrite("img_proc_Canny.png", auto)


# Wait for any keypress, then terminate and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()