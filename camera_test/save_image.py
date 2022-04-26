import pyrealsense2 as rs
import cv2
import numpy as np

# Define image size parameters
WIDTH = 640
HEIGHT = 480

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting supported resolutions
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("No color sensor detected")
    exit(0)
    
# Set the image frame size and FPS speed
# Note rs.format.z16 implements uint16 datatype
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
# Configure RGB camera input data, rs.format.bgr8 implements 8-bit red, 8-bit green, and 8-bit blue per pixel
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)

# Start streaming with the enumerated parameters
pipeline.start(config)

try:
    while True:

        # To reference specific key press
        key = cv2.waitKey(1) & 0xFF

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays and display
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow('Live stream', color_image)
		
		# If the user presses 'q', capture a still image from the stream
		# and save as 'Still_image.jpg"
        if key == ord("q"):
            cv2.imwrite("still_image.jpg", color_image)
            print("Image saved, program terminated")
            break
    
finally: 
    # Close all windows and stop the stream
    cv2.destroyAllWindows()
    pipeline.stop()
    
