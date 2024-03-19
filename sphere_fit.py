###Sidney Tsui
###lab 5
### 3/19/2024

###flipped image outline 

#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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


img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/sphere_params', Image, queue_size = 1)
	
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


