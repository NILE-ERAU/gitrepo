# Import the necessary packages
import imutils
import cv2

class AspectAwarePreprocessor:
	def __init__(self, width, height, inter=cv2.INTER_AREA):
		# Store the target image width, height, and interpolation
		# method used when resizing
		self.width = width
		self.height = height
		self.inter = inter
	
	# Define function for determing image delta offsets to be used when
	# cropping image along the larger dimension	
	def preprocess(self, image):
		# Grab the dimensions of the image and then initialize the 
		# deltas to use when cropping
		(h, w) = image.shape[:2]
		dW = 0
		dH = 0
		
		# If the width is smaller than the height, then resize along 
		# the width (i.e. the smaller dimension) and then update the
		# deltas to crop the height to the desired dimension
		if w < h: 
			image = imutils.resize(image, width=self.width, inter=self.inter)
			dH = int((image.shape[0] - self.height) / 2.0)
			
		# Otherwise, the height is smaller than the width so resize 
		# along the height and then update the deltas to crop along the 
		# width
		else:
			image = imutils.resize(image, height=self.height, inter=self.inter)
			dW = int((image.shape[1] - self.width) / 2.0)
			
		# Acquire the width and height of the resized image and crop 
		# about the center
		(h, w) = image.shape[:2]
		image = image[dH:h - dH, dW:w - dW]
		
		# Resize the image to provide spatial dimensions to ensure the 
		# output image always possesses a fixed size
		return cv2.resize(image, (self.width, self.height), interpolation=self.inter)
