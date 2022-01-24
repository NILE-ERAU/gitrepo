import cv2
import numpy as np
import os
import glob
import mahotas as mt
from sklearn.svm import LinearSVC

# function to extract haralick textures from an image
def extract_features(image):
	# calculate haralick texture features for 4 types of adjacency
	textures = mt.features.haralick(image)

	# take the mean of it and return it
	ht_mean  = textures.mean(axis=0)
	return ht_mean

# load the training dataset
train_path  = "dataset/train"
train_names = os.listdir(train_path)

# empty list to hold feature vectors and train labels
train_features = []
train_labels   = []

# loop over the training dataset
print("[STATUS] Started extracting haralick textures..")
for train_name in train_names:
	cur_path = train_path + "/" + train_name
	cur_label = train_name
	i = 1

	for file in glob.glob(cur_path + "/*.jpeg"):
		print("Processing Image - {} in {}".format(i, cur_label))
		# read the training image
		image = cv2.imread(file)

		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# extract haralick texture from the image
		features = extract_features(gray)

		# append the feature vector and label
		train_features.append(features)
		train_labels.append(cur_label)

		# show loop update
		i += 1

# have a look at the size of our feature vector and labels
print("Training features: {}".format(np.array(train_features).shape))
print("Training labels: {}".format(np.array(train_labels).shape))

# create the classifier
print("[STATUS] Creating the classifier..")
clf_svm = LinearSVC(random_state=9, max_iter = 1000)

# fit the training data and labels
print("[STATUS] Fitting data/label to model..")
clf_svm.fit(train_features, train_labels)

# loop over the test images
test_path = "dataset/test"
for file in glob.glob(test_path + "/*.jpeg"):
	# read the input image
	image = cv2.imread(file)

	# convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# extract haralick texture from the image
	features = extract_features(gray)

	# evaluate the model and predict label
	prediction = clf_svm.predict(features.reshape(1, -1))[0]

	# show the label
	cv2.putText(image, prediction, (20,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255), 3)
	print("Prediction - {}".format(prediction))

	# display the output image
	cv2.imshow("Test_Image", image)
	cv2.waitKey(0)



# # import the necessary packages
# from sklearn.svm import LinearSVC
# import argparse
# import mahotas
# import glob
# import cv2
  
# # construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-d", "--training", required=True, help="Path to the dataset of textures")
# ap.add_argument("-t", "--test", required=True, help="Path to the test images")
# args = vars(ap.parse_args())
  
# # initialize the data matrix and the list of labels
# print("[INFO] extracting features...")
# data = []
# labels = []
  
# # loop over the dataset of images
# for imagePath in glob.glob(args["training"] + "/*.png"):
    # # load the image, convert it to grayscale, and extract the texture
    # # name from the filename
    # image = cv2.imread(imagePath)
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # texture = imagePath[imagePath.rfind("/") + 1:].split("_")[0]
  
    # # extract Haralick texture features in 4 directions, then take the
    # # mean of each direction
    # features = mahotas.features.haralick(image).mean(axis=0)
  
    # # update the data and labels
    # data.append(features)
    # labels.append(texture)
  
# # train the classifier
# print("[INFO] training model...")
# model = LinearSVC(C=10.0, random_state=42)
# model.fit(data, labels)
# print("[INFO] classifying...")
 
# # loop over the test images
# for imagePath in glob.glob(args["test"] + "/*.png"):
    # # load the image, convert it to grayscale, and extract Haralick
    # # texture from the test image
    # image = cv2.imread(imagePath)
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # features = mahotas.features.haralick(gray).mean(axis=0)
  
    # # classify the test image
    # pred = model.predict(features.reshape(1, -1))[0]
    # cv2.putText(image, pred, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
        # (0, 255, 0), 3)
  
    # # show the output image
    # cv2.imshow("Image", image)
    # cv2.waitKey(0)
