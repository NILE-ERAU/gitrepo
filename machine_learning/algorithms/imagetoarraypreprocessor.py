# import the necessary packages
from keras.preprocessing.image import img_to_array

# Define Image-to-Array class for processing
class ImageToArrayPreprocessor:
	# Use default keras.json data format
	def __init__(self, dataFormat=None):
		# Store the image data format
		self.dataFormat = dataFormat
		
	def preprocess(self, image):
		# Apply the Keras utility function that correctly rearranges 
		# image dimensions
		return img_to_array(image, data_format=self.dataFormat)
		
