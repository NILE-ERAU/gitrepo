# USAGE
# python object_detection.py -i ~/Downloads -m diagnostic_model.hdf5 -c 0.8

# import the necessary packages
from algorithms.feature_extraction import image_pyramid
from algorithms.feature_extraction import sliding_window
from algorithms.feature_extraction import classify_batch
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.object_detection import non_max_suppression
import imutils
from imutils import paths
import numpy as np
import argparse
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to the input images")
ap.add_argument("-m", "--model", required=True,
	help="path to trained CNN model to import")
ap.add_argument("-c", "--confidence", type=float, default=0.5,
	help="minimum probability to filter weak detections")
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


print("[INFO] loading object detector...")
model = load_model(args["model"])

for imagePath in paths.list_images(args["image"]):
	orig = cv2.imread(imagePath)
	#orig = cv2.imread(args["image"])
	resized = resize(orig, 64, 64)
	image = img_to_array(resized) / 255.0
	image = np.expand_dims(image, axis=0)

	# Predict the bounding box of the object along with the class label
	# Note labels may be [dead leaves, sprouts, mache]
	labels = ['dead leaves', 'mache', 'sprout']
	(labelPreds) = model.predict(image)
	print("Raw predictions: {}".format(labelPreds))
	#print("Top confidence: {}".format(np.argmax(labelPreds)))

	# determine the class label with largest predicted probability
	#if np.argmax(labelPreds) > args["confidence"]: 
	pred = labels[np.argmax(labelPreds)]
	#pred = np.argmax(labelPreds, axis=1)
	#print("Prediction: {}".format(pred))
	#label = lb.classes_[i][0]

	# load the input image (in OpenCV format), resize it such that it
	# fits on our screen, and grab its dimensions
	#image = cv2.imread(imagePath)
	#image = imutils.resize(image, width=600)
	#(h, w) = image.shape[:2]
	# scale the predicted bounding box coordinates based on the image
	# dimensions
	# startX = int(startX * w)
	# startY = int(startY * h)
	# endX = int(endX * w)
	# endY = int(endY * h)
	# draw the predicted bounding box and class label on the image
	#y = startY - 10 if startY - 10 > 10 else startY + 10
	output = resize(orig, 256, 256)
	cv2.putText(output, pred, (10,20), cv2.FONT_HERSHEY_SIMPLEX,
		0.65, (255, 255, 0), 2)
	# cv2.rectangle(image, (startX, startY), (endX, endY),
		# (0, 255, 0), 2)
	# show the output image
	cv2.imshow("Output", output)
	cv2.waitKey(0)
		
	# else: 
		# print("Top prediction: {}".format(np.argmax(labelPreds)))
		# output = resize(orig, 256, 256)
		# cv2.putText(output, "Low Confidence", (10,20), cv2.FONT_HERSHEY_SIMPLEX,
			# 0.65, (255, 255, 0), 2)
		# # cv2.rectangle(image, (startX, startY), (endX, endY),
			# # (0, 255, 0), 2)
		# # show the output image
		# cv2.imshow("Output", output)
		# cv2.waitKey(0)	
	

# # initialize variables used for the object detection procedure
# #INPUT_SIZE = (350, 350)
# PYR_SCALE = 1.5
# WIN_STEP = 16
# ROI_SIZE = (64, 64)
# BATCH_SIZE = 64

# # load our the network weights from disk
# # with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
	# # print("[INFO] loading pretrained network...")
	# # model = load_model(args["model"])
# model = load_model(args["model"])

# # initialize the object detection dictionary which maps class labels
# # to their predicted bounding boxes and associated probability
# labels = {}
# #labels = ["mache", "sprout", "dead_leaves"]

# # load the input image from disk and grab its dimensions
# orig = cv2.imread(args["image"])
# (h, w) = orig.shape[:2]

# # resize the input image to be a square
# resized = resize(orig, 350, 350)
# # resized = cv2.resize(orig, INPUT_SIZE, interpolation=cv2.INTER_CUBIC)
                           
# # initialize the batch ROIs and (x, y)-coordinates
# batchROIs = None
# batchLocs = []

# # start the timer
# print("[INFO] detecting objects...")
# start = time.time()

# # loop over the image pyramid
# for image in image_pyramid(resized, scale=PYR_SCALE,
	# minSize=ROI_SIZE):
	# # loop over the sliding window locations
	# for (x, y, roi) in sliding_window(image, WIN_STEP, ROI_SIZE):
		# # take the ROI and pre-process it so we can later classify the
		# # region with Keras
		# roi = img_to_array(roi)
		# roi = np.expand_dims(roi, axis=0)
		# #roi = imagenet_utils.preprocess_input(roi)

		# # if the batch is None, initialize it
		# if batchROIs is None:
			# batchROIs = roi

		# # otherwise, add the ROI to the bottom of the batch
		# else:
			# batchROIs = np.vstack([batchROIs, roi])

		# # add the (x, y)-coordinates of the sliding window to the batch
		# batchLocs.append((x, y))

		# # check to see if our batch is full
		# if len(batchROIs) == BATCH_SIZE:
			# # classify the batch, then reset the batch ROIs and
			# # (x, y)-coordinates
			# labels = classify_batch(model, batchROIs, batchLocs,
				# labels, minProb=args["confidence"])

			# # reset the batch ROIs and (x, y)-coordinates
			# batchROIs = None
			# batchLocs = []

# # check to see if there are any remaining ROIs that still need to be
# # classified
# if batchROIs is not None:
	# labels = classify_batch(model, batchROIs, batchLocs, labels,
		# minProb=args["confidence"])

# # show how long the detection process took
# end = time.time()
# print("[INFO] detections took {:.4f} seconds".format(end - start))

# # loop over the labels for each of detected objects in the image
# for k in labels.keys():
	# print("[STATUS] Looping over labels")
	# # clone the input image so we can draw on it
	# clone = resized.copy()

	# # loop over all bounding boxes for the label and draw them on
	# # the image
	# for (box, prob) in labels[k]:
		# (xA, yA, xB, yB) = box
		# cv2.rectangle(clone, (xA, yA), (xB, yB), (0, 255, 0), 2)

	# # show the image *without* apply non-maxima suppression
	# cv2.imshow("Without NMS", clone)
	# clone = resized.copy()

	# # grab the bounding boxes and associated probabilities for each
	# # detection, then apply non-maxima suppression to suppress
	# # weaker, overlapping detections
	# boxes = np.array([p[0] for p in labels[k]])
	# proba = np.array([p[1] for p in labels[k]])
	# boxes = non_max_suppression(boxes, proba)

	# # loop over the bounding boxes again, this time only drawing the
	# # ones that were *not* suppressed
	# for (xA, yA, xB, yB) in boxes:
		# cv2.rectangle(clone, (xA, yA), (xB, yB), (0, 0, 255), 2)

	# # show the output image
	# print("[INFO] {}: {}".format(k, len(boxes)))
	# cv2.imshow("With NMS", clone)
	# cv2.waitKey(0)
