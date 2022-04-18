# Test script for initializing and interfacing with a ROS publisher
# Implementation: python3 ros_talk.py
<<<<<<< HEAD
# Invokes roscore, rosnode, and opens additional terminal in virtual
# environment for running computer vision script
=======
# Launches roscore, a serial ros node, and opens an additional terminal within a virtual environment for executing computer vision scripts
>>>>>>> e5030a6eebf2673c49c3628af8dec9e412bcba62
import subprocess

# Import ROS libraries
import rospy
import os
import time

# from std_msgs.msg import UInt16
# from std_msgs.msg import Float64
from std_msgs.msg import String
##from std_msgs.msg import Int16MultiArray


# from multiprocessing import Process,Queue,Pipe
# from cv_test import f

# if __name__ == '__main__':
    # parent_conn,child_conn = Pipe()
    # p = Process(target=f, args=(child_conn,))
    # p.start()
    # print(parent_conn.recv())   # prints "Hello"



# Define function for initializing ROS message publishing
def ros_talker(command):
    # Define ROS publisher for sending String data to topic 'move'
    pub1 = rospy.Publisher('move', String, queue_size=10)
#     pub2 = rospy.Publisher('status', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # Set rate to 10hz

      # Display to ROS console
##    rospy.loginfo(vector)
#     rospy.get_time()
#     rospy.loginfo(angle)
#    rospy.loginfo(position)
    
#     # Clean print by writing over previous message
#     sys.stdout.write(CURSOR_UP)
#     sys.stdout.write(ERASE_LINE)
    pub1.publish(command)
    # Implement delay
    rate.sleep()
    
# Run subprocess to open a new terminal and invoke the script 'ros_start' for initiating roscore and a serial node
subprocess.call(["gnome-terminal", "--","python3", "ros_start.py"])

while True:
    # Request input from user
    user = str(input("Enter stepper displacement vector: "))

    # Quit the program if the 'q' key is pressed
    # Need to add call to wait for serial node to finish transmission before killing
    if user == "q":
        # Kill the ROS serial node and allow termination of roscoren
        # If necessary, kill residual roscore node with terminal vector "kilall rosmaster"
        os.system("rosnode kill /serial_node")
##        os.system("killall rosmaster")
        print("Program terminated by user")
        break
    else:
        # Publish the user input to the ROS topic    
        ros_talker(user)
