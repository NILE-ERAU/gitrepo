import subprocess
import os
<<<<<<< HEAD
subprocess.call(["gnome-terminal", "--","python3", "virtualenv_start.py"])
=======
# Uncomment for final product: launches terminal in proper directory and
# virtual environment
#subprocess.call(["gnome-terminal", "--","python3", "virtualenv_start.py"])
>>>>>>> e5030a6eebf2673c49c3628af8dec9e412bcba62

# Launch ROS master node 'roscore'
roscore = subprocess.Popen('roscore')

# Launch rosserial node
os.system("rosrun rosserial_python serial_node.py /dev/ttyACM0")
