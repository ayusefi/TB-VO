#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visual_odometry_algorithm2 import *
from time import sleep




class TrtlbtVO:

	def __init__(self):

		# Turtlebot Pose Publisher
		self.pos_pub = rospy.Publisher('/Visual_Odometry', Twist, queue_size=10)
	#	pub = Twist()

		# Frame Subscriber
		self.bridge = CvBridge()
		self.frame_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.frm_callback)






	# Read image from ros topic
	def frm_callback(self, data):

		# Convert ROS Image to OpenCV Image
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e

		# Show OpenCV image
#		cv2.imshow("Image Window", cv_image)
#		cv2.waitKey(3)
		cv2.imwrite('messigray.png',cv_image)
		visualOdometry(cv_image)


#		try:
#			self.pos_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#		except CvBridgeError as e:
#			print(e)

	
	
	




def main():

	#Initialize Class
	clsstart = TrtlbtVO()

	# Start ROS node
	rospy.init_node('TrtlbtVO', anonymous=True)


	


	# Run node until press ctrl+c
	try:
		rospy.spin()
	except keyboardInterrupt:
		Print("Shutting Down")
	cv2.destroyAllWindows()






if __name__ == '__main__':
	main()
