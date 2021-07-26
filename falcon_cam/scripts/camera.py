#!/usr/bin/env python3

# running on python 3 - open cv 4.2.0
import numpy as np
import cv2 as cv
import csv
import time

import rospy
from std_msgs.msg import Float64
from marvelmind_nav.msg import hedge_pose_ang

# change variables here
frame_width = 640
frame_height = 480
fps = 30

class FalconVideo(object):
	def __init__(self):
		self.cap = cv.VideoCapture(0)
		self.width = self.cap.set(3,frame_width)
		self.height = self.cap.set(4,frame_height)
		# Define the codec and create VideoWriter object
		self.codec = cv.VideoWriter_fourcc(*'H264')
		self.out = cv.VideoWriter('/home/oceania/Videos/falcon/output.avi', self.codec, fps, (frame_width,  frame_height))

		self.log_file = open('/home/oceania/Videos/falcon/output.csv', 'w')
		self.file_writer = csv.writer(self.log_file)
		self.is_recording = False
		self.x = 0
		self.y = 0
		self.yaw = 0
		
		self.robot_pose_sub = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, self.robot_pose_cb, queue_size=1)
		self.robot_pose_sub = rospy.Subscriber("robot_yaw", Float64, self.robot_yaw_cb, queue_size=1)

	def recording(self):
		if not self.is_recording:
			rospy.loginfo_throttle(5,"Waiting for recording trigger...")
		else:
			self.start_time = time.time()
			while self.cap.isOpened():
				self.ret, self.frame = self.cap.read()
				if not self.ret:
					  print("Can't receive frame (stream end?). Exiting ...")
					  break
				self.frame = cv.flip(self.frame, 0)
				# write the flipped frame
				self.out.write(self.frame)
				# write to csv file
				self.current_time = time.time()
				self.timestamp = self.current_time - self.start_time
				self.file_writer.writerow([self.timestamp, self.x, self.y, self.yaw])
				print("frame "+str(i)+" received")
				i+=1
				# show image feed
				cv.imshow('frame', frame)
				if cv.waitKey(1) == ord('q'):
					  break
					  
	def robot_pose_cb(self, msg):
		self.x = msg.x
		self.y = msg.y
		
	def robot_yaw_cb(self, msg):
		self.yaw = msg.data
  	
if __name__ == "__main__":
	rospy.init_node("falcon_cam_node")
	rospy.loginfo("Initiating falcon cam node")
	rate = rospy.Rate(10)

	video = FalconVideo()
	while not rospy.is_shutdown():
		rate.sleep()
		video.is_recording = rospy.get_param("/is_recording")
		video.recording()
	# release all when ros is stopped
	cap.release()
	out.release()
	cv.destroyAllWindows()
	log_file.close()
