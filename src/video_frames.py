#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def video_publisher():
    # Start a new node named 'video_publisher'
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('video_frames', Image, queue_size=10)

    # Set the rate of publishing the frames
    rate = rospy.Rate(30) # 30 fps

    # Load the video
    cap = cv2.VideoCapture('/home/mobilio/Downloads/lane4.mp4')

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            break

        # Convert the image to ROS Image message
        ros_img = bridge.cv2_to_imgmsg(frame, "bgr8")
        
        # Publish the image.
        pub.publish(ros_img)

        rate.sleep()

    # Release the video capture
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
