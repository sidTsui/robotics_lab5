###Sidney Tsui
###lab 5
### 3/19/2024

### used flipped image outline 
#used in class lectures, and online sources

#!/usr/bin/env python3
import rospy
import numpy as np
import math 
from robot_vision_lectures.msg import XYZarray, SphereParams

##init global variables
matrix_a =[]
matrix_b=[]
P = np.array([])

# builds matrices from receiving the data points
def build_matrices(data_points):
	global matrix_a, matrix_b
	matrix_a = []
	matrix_b = []
	# create matrices from Points array
	for i in data_points.points:
		#[2*x, 2*y, 2*z, 1]
		matrix_a.append([2*point.x, 2*point.y, 2*point.z, 1])
		#[x^2 + y^2 + z^2]
		matrix_b.append([point.x**2 + point.y**2 + point.z**2])
# fits the sphere model to data points and finds P
def fit(matrix_a, matrix_b):
	global P
	A = np.array(matrix_a)
	B = np.array(matrix_b)
	try:
		ATA = np.matmul(A.T, A)
		ATB = np.matmul(A.T, B)
		P = np.matmul(np.linalg.inv(ATA), ATB)
	except np linalg.LinAlgError:
		pass
#calc sphere params from fit function finding  P 
def params(P):
	sph_params = SphereParams()
	#calc center of sphere
	sph_params.xc = P[0]
	sph_params.yc = P[1]
	sph_params.zc = P[2]
	# calc radius
	radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
	sph_params.radius = radius
	return sph_params
	
if __name__ == '__main__':
	#init ros node
	# define the node and subcribers and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to ream images
	# subscribe to /xyz_cropped topic
	img_sub = rospy.Subscriber("/xyz_cropped", XYZarray, build_matrices) 
	#create publisher sphereParameters on the /sphere_params topic
	# define a publisher to publish images
	sp_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	# set the loop frequency to 10 hz
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# check if matrices are not empty then run model_fitting
		if len(matrix_a) > 0 and len(matrix_b) > 0:
			fit(matrix_a, matrix_b)
			# check if P is not empty then run calc_sparams
			if len(P) > 0:
				sph_params = params(P)
				# publish sphere params
				sp_pub.publish(sph_params)
		rate.sleep()


