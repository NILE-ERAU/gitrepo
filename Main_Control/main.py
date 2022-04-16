# Test script for NILE robot ROS architecture and system operational
# nodes and interfacing with system website driven by mySQL
# Implementation: python3 main.py
# Launches roscore, a serial ros node, and opens an additional terminal within a virtual environment for executing computer vision scripts

# Import ROS and OS libraries
import rospy
import os
import time
import subprocess

# Import libraries for ROS message types
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import UInt16
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from std_msgs.msg import String

# Import libraries for mySQL website interfacing
import mySQL_Control as sql
import time
import datetime

# Initialize variables
homing = None
coord = Float64MultiArray(data=[0, 0, 0])
hydrate = None
sense = None
shock = Int16MultiArray(data=[0, 0, 0])

# Run subprocess to open a new terminal and invoke the script 'ros_start' for initiating roscore and a serial node
subprocess.call(["gnome-terminal", "--","python3", "ros_start.py"])	

Ts = 5
t = 0
last_t = 0
sql.assign_ip()

def print_val(data):
	i = 1
	# print("Encoder readings: {}".format(data))

# Define function for ROS publishing and subscribing based on website inputs
def ros_website(execute, d0, d1, io):
	
	# Invoke ROS publisher for topic 'home'
	if execute == "homeTrolley":
		home_pub.publish("trolley")
		
	elif execute == "homeVert":
		home_pub.publish("stepper")
		
	elif execute == "homeRot":
		home_pub.publish("rot")
	
	# Invoke ROS publisher for topic 'water'	
	elif execute == "water":
		water_pub.publish(d0)
	
	# Invoke ROS publisher for topic 'shock'	
	elif execute == "hvec":
		hvec_pub(Int16MultiArray(data=[d0, d1, io]))
		
	elif execute == "senseSoil":
		sensor_pub.publish("moisture")
		print("Read moisture sensor")
		
	# elif execute == "takeImage":
		# Capture still image from camera
		
		
	

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
	water_pub = rospy.Publisher('water', UInt16, queue_size=10)

	# Define ROS publisher for sending HVEC activation parameters to 
	# topic 'shock'
	hvec_pub = rospy.Publisher('shock', Int16MultiArray, queue_size=10)    
	
	# Define ROS subscriber for receiving joint encoder values from topic
	# 'encoder'
	encoder_sub = rospy.Subscriber('encoder', Float64MultiArray, print_val)
	
	# Define ROS subscriber for receiving soil moisture measurements
	# from topic 'moist'
	moist_sub = rospy.Subscriber('moist', UInt16, print_val)
	
	# Define ROS subscriber for receiving soil temperature measurements
	# from topic 'temp'
	temp_sub = rospy.Subscriber('temp', Float64, print_val)
	# Define ROS subscriber for receiving HVEC temperature measurements
	# from topic 'hvec'
	hvec_sub = rospy.Subscriber('hvec', Float64, print_val)

	rospy.init_node('publisher', anonymous=True)
	rate = rospy.Rate(10) # Set rate to 10hz
	rate.sleep()

# Display to ROS console
#     rospy.get_time()
#     rospy.loginfo(angle)
#     rospy.loginfo(position)
    
#     # Clean print by writing over previous message
#     sys.stdout.write(CURSOR_UP)
#     sys.stdout.write(ERASE_LINE)
	
	while True:
		
		t = time.monotonic()
		if (t - last_t >= Ts):
			timetowait = sql.time_until()
			print(timetowait)
			if (timetowait >= 0):
				queued = sql.pull_next_command()
				command = queued[2]
				theta_q = queued[3]
				r_q = queued[4]
				z_q = queued[5]
				d0_q = queued[6]
				d1_q = queued[7]
				i0_q = queued[8]				
				
				# Publish the user input to the ROS topic 
				ros_website(command, d0_q, d1_q, i0_q)
				sql.complete_command(0,0,0,"Success")
				print("Success!")
				
			last_t = t		

		# Request input from user
		var = str(input("Quit program? (y or n): "))

		# Quit the program if the 'q' key is pressed
		# Need to add call to wait for serial node to finish transmission before killing
		if var == "y":
			# Kill the ROS serial node and allow termination of roscoren
			# If necessary, kill residual roscore node with terminal vector "kilall rosmaster"
			os.system("rosnode kill /serial_node")
	        # os.system("killall rosmaster")
			print("Program terminated by user")
			break
   
