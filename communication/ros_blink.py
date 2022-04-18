# Import ROS libraries
import rospy
import os
import time
import subprocess

# from std_msgs.msg import UInt16
# from std_msgs.msg import Float64
from std_msgs.msg import String
##from std_msgs.msg import Int16MultiArray

# Define function for initializing ROS message publishing
def ros_talker(command):
    # Define ROS publisher for sending String data to topic 'move'
    pub1 = rospy.Publisher('LED', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # Set rate to 10hz
#     sys.stdout.write(CURSOR_UP)
#     sys.stdout.write(ERASE_LINE)
    pub1.publish(command)
    # Implement delay
    rate.sleep()

# Run subprocess to open a new terminal and invoke the script 'ros_start' for initiating roscore and a serial node
subprocess.call(["gnome-terminal", "--","python3", "ros_start.py"])
    
while True:
    # Request input from user
    user = str(input("LED status on or off: "))

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
