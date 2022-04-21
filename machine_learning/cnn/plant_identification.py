# Test script for evaluating trained CNN on new images

# import the necessary packages
# from algorithms.feature_extraction import image_pyramid
# from algorithms.feature_extraction import sliding_window
# from algorithms.feature_extraction import classify_batch
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.object_detection import non_max_suppression
from algorithms import ImageToArrayPreprocessor
import imutils
from imutils import paths
import numpy as np
import argparse
import time
import cv2
import pickle
import joblib

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True,
	help="directory to set of input images")
ap.add_argument("-m", "--model", required=True,
	help="path to trained CNN model to import")
# ap.add_argument("-c", "--confidence", type=float, default=0.5,
	# help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

# Define  function for resizing images while preserving aspect ratio
def resize(original, wide, tall):
	# Grab the dimensions of the image and then initialize the 
	# deltas to use when cropping
	(h, w) = original.shape[:2]
	dW = 0
	dH = 0
	
	# If the width is smaller than the height, then resize along 
	# the width (i.e. the smaller dimension) and then update the
	# deltas to crop the height to the desired dimension
	if w < h: 
		image = imutils.resize(original, width=wide, inter=cv2.INTER_AREA)
		dH = int((image.shape[0] - original.shape[0]) / 2.0)
		
	# Otherwise, the height is smaller than the width so resize 
	# along the height and then update the deltas to crop along the 
	# width
	else:
		image = imutils.resize(original, height=tall, inter=cv2.INTER_AREA)
		dW = int((image.shape[1] - original.shape[1]) / 2.0)
		
	# Acquire the width and height of the resized image and crop 
	# about the center
	(h, w) = image.shape[:2]
	image = image[dH:h - dH, dW:w - dW]
	
	# Resize the image to provide spatial dimensions to ensure the 
	# output image always possesses a fixed size
	return cv2.resize(image, (wide, tall), interpolation=cv2.INTER_AREA)

# Load pre-trained model and desired image
model = load_model(args["model"])
#model = joblib.load(args["model"])

# randomly sample a few of the input images
imagePaths = list(paths.list_images(args["images"]))
imagePaths = np.random.choice(imagePaths, size=(59,), replace=False)

for imagePath in imagePaths:
	# load and resize the image
	image = cv2.imread(imagePath)
	image = resize(image, 64, 64)
	
	cv2.imshow("Sample", image)
	cv2.waitKey()

	# Convert image to array
	#image = img_to_array(image)
	image = np.expand_dims(image, axis=0)
	
	# Keras make predictions
	pred = model.predict(image)[0]
	print("Prediction: {}".format(pred))
	index = np.argsort(pred)[::-1][:3]
	# label = le.inverse_transform(index[0])
	# label = label.replace(":", " ")
	# print(label)
	# pred = model.predict(image).argmax(axis=1)[0]
	# predictions.append(str(pred))
	
	# # Draw the prediction on the output image
	# cv2.putText(image, str(pred), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

	# # Display the output image
	# cv2.imshow("Output", image)
	# cv2.waitKey()

