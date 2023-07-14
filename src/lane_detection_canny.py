#!/usr/bin/env python
import roslib,rospy,sys,cv2,time

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

bridge = CvBridge()
pub = rospy.Publisher('lane_detection_canny_msg', Int32, queue_size=1) #ros-lane-detection
pub_image = rospy.Publisher('lane_detection_canny',Image, queue_size=1)


def callback(data):

	# convert image to cv2 standard format
	#img = bridge.compressed_imgmsg_to_cv2(data)
	img = bridge.imgmsg_to_cv2(data)

	# start time
	start_time = cv2.getTickCount()

	# Gaussian Filter to remove noise
	img = cv2.medianBlur(img,5)
	print(img.shape)

	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (5,5), 0)
	canny = cv2.Canny(blur, 50, 200) # low_threshold, high_threshold


	mask = np.zeros_like(img)
	if len(img.shape) > 2:
		channel_count = img.shape[2]
		ignore_mask_color = (255,) * channel_count
	else:
		ignore_mask_color = 255

		
	image = bridge.cv2_to_imgmsg(canny, "mono8")
	pub_image.publish(image)

def lane_detection():
	rospy.init_node('lane_detection_canny',anonymous=True)
	rospy.Subscriber("/video_frames",Image,callback,queue_size=1,buff_size=2**24)
	try:
		rospy.loginfo("Enetering ROS Spin")
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	try:
		lane_detection()
	except rospy.ROSInterruptException:
		pass
