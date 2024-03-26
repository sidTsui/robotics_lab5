###Sidney Tsui
###lab 5
### 3/19/2024

### used flipped image outline 

#!/usr/bin/env python3
import rospy
import numpy as np
import math 
from robot_vision_lectures.msg import XYZarray, SphereParams


def build_matrices(data_points):
	global matrix_a = []
 	global matrix_b = []
	# create matrices from Points array
	for i in data_points.points:
		matrix_a.append([2*point.x, 2*point.y, 2*point.z, 1])
		matrix_b.append([point.x**2 + point.y**2 + point.z**2])
	return(nparray(matrix_a), np.array(matrix_b))
	
def fit(matrix_a, matrix_b):
	A = np.array(matrix_a)
	B = np.array(matrix_b)
	try:
		ATA = np.matmul(A.T, A)
		ATB = np.matmul(A.T, B)
		P = np.matmul(np.linalg.inv(ATA), ATB)
		return P
	except np linalg.LinAlgError:
		return None

def params(P):
	sph_params.xc = P[0]
	sph_params.yc = P[1]
	sph_params.zc = P[2]

	radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
    	sph_params.radius = radius
	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/xyz_cropped", XYZarray, build_matrices) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# check if matrices are not empty then run model_fitting
		if len(matrix_a) > 0 and len(matrix_b) > 0:
			P = model_fitting(matrix_a, matrix_b)
			# check if P is not empty then run calc_sparams
			if P is not None:
				sph_params = params(P)
				# publish sphere params
				sp_pub.publish(sph_params)
		rate.sleep()


