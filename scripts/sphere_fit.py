###Sidney Tsui
###lab 5
### 3/19/2024

### used flipped image outline 

#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

#defining matrix A
A = np.vstack([x, np.ones(len(x))]).T
#defining matrix B
B = np.array([y]).T
#product of A^T*A
ATA = np.matmul(A.T, A)
#product of A^T*B
ATB = np.matmul(A.T, B)
product = np.matmul(np.linalg.inv(ATA),ATB)
m1 = product[0]
c1 = product[1]


def build_matrices(data_points):
	matrix_a
 	matrix_b
	# create matrices from Points array
	for i in data_points.points:
		matrix_a.append([2*point.x, 2*point.y, 2*point.z, 1])
		matrix_b.append([point.x**2 + point.y**2 + point.z**2])
		
def calc_abp(matrix_a, matrix_b):
	A = np.array(matrix_a)
	B = np.array(matrix_b)
	try:
		ATA = np.matmul(A.T, A)
		ATB = np.matmul(A.T, B)
		P = np.matmul(np.linalg.inv(ATA), ATB)
		return P
	except:
		return null

def Sphere_Params(P, S):
	
if __name__ == '__main__':
	
	# define the node and subcribers and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/xyz_cropped", XYZarray, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/sphere_params', S, queue_size = 10)
	S = SphereParams()
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			# flip the image up			
			flipped_img = cv2.flip(rgb_img, 0)
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(flipped_img, encoding="rgb8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()


