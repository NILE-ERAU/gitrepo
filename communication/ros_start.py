import subprocess
import os
subprocess.call(["gnome-terminal", "--","python3", "cv_test.py"])

# Launch ROS master node 'roscore'
roscore = subprocess.Popen('roscore')

# Launch rosserial node
os.system("rosrun rosserial_python serial_node.py /dev/ttyACM0")
