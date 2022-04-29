import subprocess
import os
# Uncomment for final product: launches terminal in proper directory and
# virtual environment
#subprocess.call(["gnome-terminal", "--","python3", "virtualenv_start.py"])

# Launch ROS master node 'roscore'
roscore = subprocess.Popen('/opt/ros/melodic/bin/roscore')
#os.system('/opt/ros/melodic/bin/roscore -v')
# Launch rosserial node
os.system("/opt/ros/melodic/bin/rosrun rosserial_python serial_node.py /dev/ttyACM0")
