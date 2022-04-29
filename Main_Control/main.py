# Test script for NILE robot ROS architecture and system operational
# nodes and interfacing with system website driven by mySQL
# Implementation: python3 main.py
# Launches roscore, a serial ros node, and opens an additional terminal within a virtual environment for executing computer vision scripts

import os


# Import libraries for RealSense camera
print('REALSENSE LIBS')
import pyrealsense2 as rs
import cv2
import numpy as np

# Import ROS and OS libraries
print('ROS LIBS')
import rospy
import time
import subprocess
import math

# Import libraries for ROS message types
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import UInt16
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

print('SQL LIBS')
# Import libraries for mySQL website interfacing
import mySQL_Control as sql
import time
import datetime
import base64

#ros_start
#time.sleep(20)

# Initialize camera parameters
# Define image size parameters
WIDTH = 640
HEIGHT = 480

print("Camera Stuff")
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting supported resolutions
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
print("After pipeline")

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("No color sensor detected")
    #exit(0)

# Configure RGB camera input data, rs.format.bgr8 implements 8-bit red, 8-bit green, and 8-bit blue per pixel
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
print("Enable Stream")
# Initialize ROS and MySQL variables
homing = None
coord = Float64MultiArray(data=[0, 0, 0])
hydrate = None
sense = None
shock = Int16MultiArray(data=[0, 0, 0])
sql_busy = False
Ts = 1
t = 0
last_t = 0
recent_complete = False
complete_ = True
temp_flag = False
moist_flag = False
live_ = False
theta = 0
r = 0
z = 0

temp = 0
moist = 0

print("ROS Subroutine")
# Run subprocess to open a new terminal and invoke the script 'ros_start' for initiating roscore and a serial node
<<<<<<< HEAD
time.sleep(2)
#subprocess.call(["python3", "/home/pyimagesearch/gitrepo/Main_Control/ros_start.py"])
#subprocess.call(["lxterm", "-e","/usr/bin/python3", "/home/pyimagesearch/gitrepo/Main_Control/ros_start.py"])
#subprocess.call(["lxterm", "-e","/opt/ros/melodic/bin/roscore"])
subprocess.call(["gnome-terminal", "--","python3", "/home/pyimagesearch/gitrepo/Main_Control/ros_start.py"])
time.sleep(2)
print("SQL IP")
=======
subprocess.call(["gnome-terminal", "--","python3", "ros_start.py"])

<<<<<<< HEAD
=======

>>>>>>> 6f96d9d4c97d6077644bf26b61a444a97e12add5
>>>>>>> 6b315caafc4cedf9cb1c42fb1267f5858b178aa9
sql.assign_ip()
time.sleep(8)


def publish_web_coords(data):
	global theta
	global r
	global z
	theta = data.data[0]*180/math.pi
	r = data.data[1]
	z = data.data[2]
	

def set_complete_flag(data):
	global complete_
	global recent_complete
	complete_ = True
	recent_complete = True
	print("Complete Flag")
	print(data.data)


def soil_moist(data):
	global moist
	global moist_flag
	moist = data.data
	moist_flag = True
	print(moist)


def soil_temp(data):
	global temp
	global temp_flag
	temp = data.data
	temp_flag = True
	print(temp)

def publish_web_sense():
	global temp_flag
	global moist_flag
	if (temp_flag and moist_flag):
		temp_flag = False
		moist_flag = False
		sql_busy = True
		sql.publish_soil_sample(theta,r,z,moist,temp)
		sql_busy = False


def print_val(data):
	i = 1
	# print("Encoder readings: {}".format(data))

# Define function for ROS publishing and subscribing based on website inputs
def ros_website(execute, theta, r, z, d0, d1, io):
	global sql_busy
	global complete_
	global recent_complete
	# Invoke ROS publisher for topic 'home'
	if execute == "homeTrolley":
		home_pub.publish("trolley")

	elif execute == "homeStepper":
		home_pub.publish("stepper")

	elif execute == "homeRot":
		home_pub.publish("rot")
	# Invoke ROS publisher for topic 'movement'
	elif execute == "moveTo":
		print('publisher')
		move_pub.publish(Float64MultiArray(data=[theta,r,z]))

	# Invoke ROS publisher for topic 'water'
	elif execute == "water":
		water_pub.publish(float(d0))

	# Invoke ROS publisher for topic 'shock'
	elif execute == "hvec":
		hvec_pub.publish(Int16MultiArray(data=[int(d0), int(d1), int(io)]))

	# Invoke ROS publisher for topic 'senseSoil'
	elif execute == "senseSoil":
		sensor_pub.publish("senseSoil")
		print("Read moisture and temperature sensor")

	# Capture still image from camera
	elif execute == "takeImage":
		# Start streaming with the enumerated parameters
		sql_busy = True
		profile = pipeline.start(config)
		
		sensor_rgb = profile.get_device().query_sensors()[1]
		sensor_rgb.set_option(rs.option.exposure,200.000)
		#sensor_rgb.set_option(rs.option.enable_auto_exposure,True)
		
		# Wait for a coherent pair of frames: depth and color
		frames = pipeline.wait_for_frames()
		color_frame = frames.get_color_frame()

		# Convert image to numpy array and save as JPG
		color_image = np.asanyarray(color_frame.get_data())
		cv2.imwrite("test_image.jpg", color_image)
		print("Image saved")
		pipeline.stop()

		with open("/home/pyimagesearch/gitrepo/Main_Control/test_image.jpg", "rb") as image_file:
			encoded_string = base64.b64encode(image_file.read())
		#encoded_string = base64.b64encode(color_image)
		
		sql.publish_live_image(encoded_string,theta,r,z)
		
		# Re-set appropriate flags
		complete_ = True
		recent_complete = True
		sql_busy = False
	elif execute == "goLive":
		global live_
		complete_ = True
		recent_complete = True
		if (live_):
			live_ = False
		else:
			live_ = True
		# Start streaming with the enumerated parameters
		

# Main loop
if __name__ == '__main__':

	# Initialize ROS publishers and subscribers
	# Define ROS publisher for sending string data to topic 'home'
	home_pub = rospy.Publisher('home', String, queue_size=10)

	# Define ROS publisher for sending string data to topic 'coordinates'
	move_pub = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)

	# Define ROS publisher for sending selector string for sensor to
	# topic 'sensor'
	sensor_pub = rospy.Publisher('sensor', String, queue_size=10)

	# Define ROS publisher for sending integer data to topic 'water'
	water_pub = rospy.Publisher('water', Float64, queue_size=10)

	# Define ROS publisher for sending HVEC activation parameters to
	# topic 'shock'
	hvec_pub = rospy.Publisher('shock', Int16MultiArray, queue_size=10)

	# Define ROS subscriber for receiving joint encoder values from topic
	# 'encoder'
	encoder_sub = rospy.Subscriber('encoder', Float64MultiArray, publish_web_coords)

	# Define ROS subscriber for receiving soil moisture measurements
	# from topic 'moist'
	moist_sub = rospy.Subscriber('moist', UInt16, soil_moist)

	# Define ROS subscriber for receiving soil temperature measurements
	# from topic 'temp'
	temp_sub = rospy.Subscriber('temp', Float64, soil_temp)

	# Define ROS subscriber for receiving command completeness
	# from topic 'complete'
	complete_sub = rospy.Subscriber('complete', UInt16, set_complete_flag)


	rospy.init_node('publisher', anonymous=True)

	#rospy.init_node('listener', anonymous=True)
	rate = rospy.Rate(10) # Set rate to 10hz
	rate.sleep()

	while True:

		t = time.monotonic()
		if (t - last_t >= Ts):
			last_t = t
			publish_web_sense()
			if (not sql_busy):
				sql_busy = True
				timetowait = sql.time_until()
				sql_busy = False
				print(timetowait)
				sql_busy = True
				sql.publish_pos(theta, r, z)
				sql_busy = False
				if (live_):
					sql_busy = False
					profile = pipeline.start(config)
					
					sensor_rgb = profile.get_device().query_sensors()[1]
					sensor_rgb.set_option(rs.option.exposure,200.000)
					#sensor_rgb.set_option(rs.option.enable_auto_exposure,True)
					
					# Wait for a coherent pair of frames: depth and color
					frames = pipeline.wait_for_frames()
					color_frame = frames.get_color_frame()

					# Convert image to numpy array and save as JPG
					color_image = np.asanyarray(color_frame.get_data())
					cv2.imwrite("test_image.jpg", color_image)
					print("Image saved")
					pipeline.stop()
					with open("/home/pyimagesearch/gitrepo/Main_Control/test_image.jpg", "rb") as image_file:
						encoded_string = base64.b64encode(image_file.read())
					sql_busy = True
					sql.publish_live_image(encoded_string,theta,r,z)
					sql_busy = False
				if (recent_complete):
					recent_complete = False
					complete_ = True
					sql.complete_command(theta,r,z,"Success")
					print("Success!")
			if (timetowait >= 0 and complete_ and not sql_busy):
				sql_busy = True
				queued = sql.pull_next_command()
				sql_busy = False
				if (queued[0] == 0):
					print('SQL Command Retrieve Failure')
				else:
					command = queued[2]
					theta_q = queued[3]*math.pi/180
					r_q = queued[4]
					z_q = queued[5]
					d0_q = queued[6]
					d1_q = queued[7]
					i0_q = queued[8]

					# Publish the user input to the ROS topic
					print(command)
					complete_ = False
					if (command == "kill"):
						sql_busy = True
						sql.complete_command(theta,r,z,"Success")
						sql_busy = False
						os.system("rosnode kill /serial_node")
						# os.system("killall rosmaster")
						print("Program terminated by user")
						break

					ros_website(command, theta_q, r_q, z_q, d0_q, d1_q, i0_q)
