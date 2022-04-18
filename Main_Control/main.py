# Test script for NILE robot ROS architecture and system operational
# nodes and interfacing with system website driven by mySQL
# Implementation: python3 main.py
# Launches roscore, a serial ros node, and opens an additional terminal within a virtual environment for executing computer vision scripts

# Import ROS and OS libraries
import rospy
import os
import time
import subprocess
import math

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
complete_ = True;
sql_busy = False;

sql.assign_ip()

def publish_web_coords(data):
	theta = data.data[0]*180/math.pi
	r = data.data[1]
	z = data.data[2]
    if (not sql_busy):
        sql_busy = True
        sql.publish_pos(theta, r, z)
        sql_busy = False


def set_complete_flag(data):
	complete_ = True
    while (sql_busy):
        pass
    sql_busy = True
	sql.complete_command(0,0,0,"Success")
    sql_busy = False
	print("Success!")






def print_val(data):
	i = 1
	# print("Encoder readings: {}".format(data))

# Define function for ROS publishing and subscribing based on website inputs
def ros_website(execute, theta, r, z, d0, d1, io):

	# Invoke ROS publisher for topic 'home'
	if execute == "homeTrolley":
		home_pub.publish("trolley")

	elif execute == "homeVert":
		home_pub.publish("stepper")

	elif execute == "homeRot":
		home_pub.publish("rot")
	# Invoke ROS publisher for topic 'movement'
	elif execute == "moveTo":
		print('publisher')
		move_pub.publish(Float64MultiArray(data=[theta,r,z]))

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
	encoder_sub = rospy.Subscriber('encoder', Float64MultiArray, publish_web_coords)

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
	complete_sub = rospy.Subscriber('complete', UInt16, set_complete_flag)

	rospy.init_node('publisher', anonymous=True)
	#rospy.init_node('listener', anonymous=True)
	#rospy.spin()
	rate = rospy.Rate(10) # Set rate to 10hz
	rate.sleep()

# Display to ROS console
#     rospy.get_time()
#     rospy.loginfo(angle)
#     rospy.loginfo(position)

#     # Clean print by writing over previous message
#     sys.stdout.write(CURSOR_UP)
#     sys.stdout.write(ERASE_LINE)
	time.sleep(8)
	while True:

		t = time.monotonic()
		if (t - last_t >= Ts):
            if (not sql_busy):
                sql_busy = True
                timetowait = sql.time_until()
                sql_busy = False
                print(timetowait)
			if (timetowait >= 0 and complete_ and not sql_busy):
                sql_busy = True
				time.sleep(0.1)
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
					ros_website(command, theta_q, r_q, z_q, d0_q, d1_q, i0_q)



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
