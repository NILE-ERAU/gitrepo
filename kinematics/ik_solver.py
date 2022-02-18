# Import numerical python and mathematics libraries
import numpy as np
import math

# Define pi value
pi = math.pi

# Define inverse kinematics function for system camera based on 
# algebraic forward kinematics
# Input: IIrCam task-space coordinates vector [x; y; z] in mm
# Output: cylindrical joint-space coordinates [theta, gantry, depth]

# Note: HOME IIrCam is [111.35, 227.5, 872.5] mm with respect to the 
# inertial reference frame
def cam_ik(IIrCam):
	phi = -math.atan2(IIrCam[0], IIrCam[1])
	theta_cam = math.asin((111.35)/math.sqrt(IIrCam[0]**2 + IIrCam[1]**2))
	theta = phi + theta_cam
	gantry = math.sqrt(IIrCam[0]**2 + IIrCam[1]**2) * math.cos(theta_cam) - 227.5
	depth = 1425 - 577.5 + 25 - IIrCam[2]
	
	return theta, gantry, depth

# Note: Home IIrNoz is...	
def nozzle_ik(IIrNoz):
	phi = -math.atan2(IIrCam[0], IIrCam[1])
	theta_cam = math.asin((40)/math.sqrt(IIrCam[0]**2 + IIrCam[1]**2))
	theta = phi + theta_cam
	gantry = math.sqrt(IIrCam[0]**2 + IIrCam[1]**2) * math.cos(theta_cam) - 227.5
	depth = 1425 - 577.5 - 30 - IIrCam[2]
	
	return theta, gantry, depth
	
	
# Create input task-space vector in mm	
test_loc = np.array([300, 200, 100])
t, g, d = cam_ik(test_loc)

print("Theta angle: {} radians".format(t))
print("Gantry position: {} mm".format(g))
print("End-effector depth: {} mm".format(d))
