# Implementation: python img_contour.py -i mache_sprout.jpg

# Import the necessary packages
import argparse
import cv2
import imutils

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, required=False,
	help="path to input image")
args = vars(ap.parse_args())

if not args.get("image", False):

    # If no user-supplied path, load the a preset image
    image = cv2.imread("/home/pyimagesearch/gitrepo/camera_test/still_image.jpg")

else:
    # Otherwise, use the user-supplied path
    image = cv2.imread(args["image"])
    
# Display the original image
#cv2.imshow("Original", image)
result = image.copy()
HEIGHT, WIDTH = result.shape[:2]
# print("Width: {}, Height: {}".format(WIDTH, HEIGHT))

# Define HSV boundaries for desired color
greenLower = (40, 80, 10)
greenUpper = (80, 255, 255)

blueLower = (50, 80, 10)
blueUpper = (100, 255, 255)

# Define RGB red color
red = (0, 0, 255)

# Apply Gaussian Blur and convert to HSV colorspace
blurred = cv2.GaussianBlur(image, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

# Apply color mask, with erosion-dilation sequence to eliminate tiny blotches
masked = cv2.inRange(hsv, greenLower, greenUpper)
masked = cv2.erode(masked, None, iterations=2)
masked = cv2.dilate(masked, None, iterations=4)

# Find contours in the masked image and draw a bounding box
contours = cv2.findContours(masked.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)
centroids = []
count = len(contours)
# Only proceed if at least one countour was found
if len(contours) > 0:
    # Draw bounding boxes around identified contours
    for cntr in contours:
        x,y,w,h = cv2.boundingRect(cntr)
        cv2.rectangle(result, (x,y), (x+w, y+h), red, 2)
        
        # Calculate the centroid of each bounding box with respect to 
        # the center of the camera frame
        # Note positive X is to the right of the frame's center
        # Note positive Y is above the frame's center
        center = ((x + w)//2 - WIDTH//2, (y + h)//2 - HEIGHT//2)
        centroids.append(center)

else:
    print("No contours found!")

# Display final results
# cv2.putText(result, "Mache", (0, 100), cv2.FONT_HERSHEY_SIMPLEX,
# 4.0, (0, 0, 255), 5)	
print("Number of plants found: {}".format(count))
print("Plant Centroid Locations: {}".format(centroids))
cv2.imshow("Contours", result)
cv2.imshow("Color Masked", masked)

#cv2.imwrite("img_proc_blurred.jpeg", blurred)
#cv2.imwrite("img_proc_hsv.jpeg", hsv)
##cv2.imshow("HSV Colorspace", hsv)
# cv2.imwrite("img_proc_original.png", image)
# cv2.imwrite("img_proc_HSV.png", hsv)
#cv2.imwrite("img_proc_Masked.png", masked)
#cv2.imwrite("img_proc_Contours.jpeg", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
