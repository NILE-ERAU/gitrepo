# import the necessary packages
from texture_class import LocalBinaryPatterns
from sklearn.svm import LinearSVC
from imutils import paths
import argparse
import cv2
import os
import joblib

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--training", required=True,
	help="path to the training images")
ap.add_argument("-e", "--testing", required=True, 
	help="path to the tesitng images")
args = vars(ap.parse_args())

# initialize the local binary patterns descriptor along with
# the data and label lists
desc = LocalBinaryPatterns(24, 8)
data = []
labels = []

# loop over the training images
for imagePath in paths.list_images(args["training"]):
	# load the image, convert it to grayscale, and describe it
	image = cv2.imread(imagePath)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	hist = desc.describe(gray)
	# extract the label from the image path, then update the
	# label and data lists
	labels.append(imagePath.split(os.path.sep)[-2])
	data.append(hist)
# train a Linear SVM (State Vector Model) on the data
# create the classifier
print("[STATUS] Creating the classifier..")
model = LinearSVC(C=100.0, random_state=42)
# fit the training data and labels
print("[STATUS] Fitting data/label to model..")
model.fit(data, labels)

# Save the model to disk file
# filename = 'trained_texturemodel.sav'
# joblib.dump(model, filename)

# loop over the testing images
for imagePath in paths.list_images(args["testing"]):
	# load the image, convert it to grayscale, describe it,
	# and classify it
	image = cv2.imread(imagePath)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	hist = desc.describe(gray)
	prediction = model.predict(hist.reshape(1, -1))
	
	# display the image and the prediction
	cv2.putText(image, prediction[0], (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
		1.0, (0, 0, 255), 3)
	cv2.imshow("Image", image)
	cv2.waitKey(0)
