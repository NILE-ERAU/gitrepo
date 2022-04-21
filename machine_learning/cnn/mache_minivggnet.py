# Usage:
# python mache_minivggnet.py -d ../dataset/ -m diagnostic_model.hdf5

# import the necessary packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
from algorithms import ImageToArrayPreprocessor
from algorithms import AspectAwarePreprocessor
from algorithms import SimpleDatasetLoader
from algorithms import MiniVGGNet
from keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.optimizers import SGD
#from keras.optimizers import SGD
from imutils import paths
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

# Construct the argumetn parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True, 
help="path to input dataset")
ap.add_argument("-m", "--model", required=True, 
help="path to output trained model")
args = vars(ap.parse_args())

# Grab the list of images that will be described, then extract the class
# label names from the image paths
print("[INFO] Loading images...")
imagePaths = list(paths.list_images(args["dataset"]))
classNames = [pt.split(os.path.sep)[-2] for pt in imagePaths]
classNames = [str(x) for x in np.unique(classNames)]

# class_count = len(classNames)
# print("[INFO] Number of classes: {}".format(class_count))

# Initialize the image preprocessors
aap = AspectAwarePreprocessor(64, 64)
iap = ImageToArrayPreprocessor()

# Load the dataset from disk then scale the raw pixel intensities to
# the range [0, 1] instead of [0, 255]
sdl = SimpleDatasetLoader(preprocessors=[aap, iap])
(data, labels) = sdl.load(imagePaths, verbose=200)
data = data.astype("float") / 255.0
print("[INFO] Number of images in dataset: {}".format(len(labels)))

# Partition the data into training and testing splits using 75% of the 
# data for training and the remaining 25% for testing
(trainX, testX, trainY, testY) = train_test_split(data, labels, 
test_size=0.25, random_state=42)

#print("Length of trainX; {}".format(len(trainX)))
# Convert the labels from integers to vectors
trainY = LabelBinarizer().fit_transform(trainY)
testY = LabelBinarizer().fit_transform(testY)

# Construct the image generator for data augmentation
aug = ImageDataGenerator(rotation_range=30, width_shift_range=0.1, 
height_shift_range=0.1, shear_range=0.2, zoom_range=0.2,
horizontal_flip=True, fill_mode="nearest")

# Initialize the Stochastic Gradient Descent learning rate optimizer
# Set the learning rate to 0.05
# initialize the optimizer and model
print("[INFO] compiling model...")
opt = SGD(lr=0.05)
model = MiniVGGNet.build(width=64, height=64, depth=3,
	classes=len(classNames))
model.compile(loss="categorical_crossentropy", optimizer=opt,
	metrics=["accuracy"])

# train the network over 30 epochs with data augmentation
print("[INFO] training network...")
H = model.fit_generator(aug.flow(trainX, trainY, batch_size=32),
	validation_data=(testX, testY), steps_per_epoch=len(trainX) // 32,
	epochs=30, verbose=1)	
	
# train the network without data augmentation	
# H = model.fit(trainX, trainY, validation_data=(testX, testY),
	# batch_size=32, epochs=30, verbose=1)
	
# Save the network to disk
print("[INFO] serializing network...")
model.save(args["model"])

# Evaluate the CNN's overall performance
print("[INFO] Evaluating the network...")
predictions = model.predict(testX, batch_size=32)
print(classification_report(testY.argmax(axis=1), 
	predictions.argmax(axis=1), target_names=classNames))
	
# print(classification_report(testY.argmax(axis=1), 
	# predictions.argmax(axis=1), target_names=["mache", "sprout"]))

# Plot the training and validation loss and accuracy
plt.style.use("ggplot")
plt.figure()
plt.plot(np.arange(0, 30), H.history["loss"], label="train_loss")
plt.plot(np.arange(0, 30), H.history["val_loss"], label="val_loss")
plt.plot(np.arange(0, 30), H.history["acc"], label="train_acc")
plt.plot(np.arange(0, 30), H.history["val_acc"], label="val_acc")
plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.show()
