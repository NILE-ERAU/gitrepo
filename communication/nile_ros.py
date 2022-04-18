# Test script for NILE robot ROS architecture and system operational
# nodes
# Implementation: python3 nile_ros.py
# Launches roscore, a serial ros node, and opens an additional terminal within a virtual environment for executing computer vision scripts
import subprocess

# Import ROS libraries
import rospy
import os
import time

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import UInt16
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from std_msgs.msg import String

# from multiprocessing import Process,Queue,Pipe
# from cv_test import f

# if __name__ == '__main__':
    # parent_conn,child_conn = Pipe()
    # p = Process(target=f, args=(child_conn,))
    # p.start()
    # print(parent_conn.recv())   # prints "Hello"
    
homing = None
coord = Float64MultiArray(data=[0, 0, 0])
hydrate = None
sense = None
#shock = None
shock = Int16MultiArray(data=[0, 0, 0])

# Run subprocess to open a new terminal and invoke the script 'ros_start' for initiating roscore and a serial node
subprocess.call(["gnome-terminal", "--","python3", "ros_start.py"])	

def print_val(data):
	i = 1
	# print("Encoder readings: {}".format(data))

# Define function for ROS publishing and subscribing
def ros_talker(homing, coord, hydrate, shock, sense):
	if (homing is not None):
		home_pub.publish(homing)
			
	if (coord is not None):
		move_pub.publish(coord)
		
	if (sense is not None):
		sensor_pub.publish(sense)
		
	# if (hydrate is not None):
		# water_pub.publish(hydrate)
		
	if (shock is not None):
		hvec_pub.publish(shock)

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

	# # Define ROS publisher for sending integer data to topic 'water'
	# water_pub = rospy.Publisher('water', UInt16, queue_size=10)

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
	#hvec_sub = rospy.Subscriber('hvec', Float64, print_val)
	# Define ROS subscriber for receiving command completeness
	# from topic 'complete'
	complete_sub = rospy.Subscriber('complete', UInt16, print_val)
	
	
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
		# Request input from user
		command = str(input("Enter system sensor to read: "))

		# Quit the program if the 'q' key is pressed
		# Need to add call to wait for serial node to finish transmission before killing
		if command == "q":
			# Kill the ROS serial node and allow termination of roscoren
			# If necessary, kill residual roscore node with terminal vector "kilall rosmaster"
			os.system("rosnode kill /serial_node")
	##        os.system("killall rosmaster")
			print("Program terminated by user")
			break
		else:
			# Publish the user input to the ROS topic    
			ros_talker(homing, coord, hydrate, shock, command)
