#!/usr/bin/env python3

import sys
sys.path.insert(1, '/home/oceania/falcon_ws/src/falcon_drivers/scripts/ros_drivers/rpi_serial')
from roboclaw_3 import Roboclaw

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# m1 = left motor
# m2 = right motor

class MotorDriver(object):
	def __init__(self):
		self.is_open = False
		self.rc = Roboclaw("/dev/ttyAMA0" , 38400)
		if self.rc.Open():
			self.is_open = True
			rospy.loginfo("Roboclaw port successfully opened")
			self.address = 0x80
			self.version = self.rc.ReadVersion(self.address)
		else:
			rospy.logerr("Roboclaw port not opened successfully. Please check the port name or baud rate.")
		self.ENCODER_CPR = 20  #encoder CPR
		self.GEAR_RATIO = 391 # Gear-ratio
		self.PI = 3.14 
		self.L = 1.6 # wheel base
		self.R = 0.4 # Radius of the wheel

		self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
		self.wheel_odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)

		self.odom_msg = Odometry()

		# setting to 0 
		self.w_qppsl = 0;
		self.w_qppsr = 0;

	def cmd_vel_cb(self, msg):
		self.v_x = msg.linear.x
		self.v_z = msg.angular.z
		self.w_left = ((2 * self.v_x) - (self.v_z * self.L)) / (2 * self.R)  #wheel left
		self.w_right = ((2 * self.v_x) + (self.v_z * self.L)) / (2 * self.R) #wheel right 
		self.w_qppsl = (self.w_left * self.ENCODER_CPR * self.GEAR_RATIO)/(2 * self.PI)
		self.w_qppsr = (self.w_right * self.ENCODER_CPR * self.GEAR_RATIO)/(2 * self.PI)
		self.write_speed("left", int(self.w_qppsl)) 
		self.write_speed("right", int(self.w_qppsr)) 
		rospy.loginfo("Writing speeds to motors - Left: %2f, Right: %2f", self.w_left, self.w_right)
		rospy.loginfo("qpps to motors - Left: %d, Right: %d", int(self.w_qppsl), int(self.w_qppsr))

	def write_speed(self, motor, speed):
		if self.is_open:
			self.version = self.rc.ReadVersion(self.address)
			if motor == "left":
				self.rc.SpeedM1(self.address, speed)
				print("left qpps:" + str(speed))
			elif motor == "right":
				self.rc.SpeedM2(self.address, speed)
				print("right qpps:" + str(speed))	
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to write data.")

	def publish_wheel_odom(self):
		if self.is_open:
			self.read_encoder()
			self.read_speed()
			self.odom_msg.pose.pose.position.x = 1
			self.wheel_odom_pub.publish(self.odom_msg)
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to retrieve data.")

	def read_encoder(self):
		if self.is_open:
			self.enc_l = self.rc.ReadEncM1(self.address)
			self.enc_r = self.rc.ReadEncM2(self.address)
			
			#print("Left encoder:")
			if(self.enc_l[0]==1):
				pass
				#print(self.enc_l[1])
				#print(self.enc_l[2])
				# convert from qpps to position
			else:
				rospy.logwarn_throttle(1, "Invalid left encoder received from roboclaw")
			#print("Right encoder:")
			if(self.enc_r[0]==1):
				pass
				#print(self.enc_r[1])
				#print(self.enc_r[2])
			else:
				rospy.logwarn_throttle(1, "Invalid right encoder received from roboclaw")
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to retrieve data.")

	def read_speed(self):
		if self.is_open:
			self.speed_qppsl = self.rc.ReadSpeedM1(self.address)
			self.speed_qppsr = self.rc.ReadSpeedM2(self.address)
			
			if(self.speed_qppsl[0]):
				print("Left wheel qpps:"+str(self.speed_qppsl[1]))
				# convert from qpps to velocity
				self.speed_left = (self.speed_qppsl[1]*60)/(self.ENCODER_CPR*self.GEAR_RATIO)
				print("Left wheel rpm: " + str(self.speed_left))
				print("Sent left qpps:" + str(self.w_qppsl))
			else:
				rospy.logwarn_throttle(1, "Invalid left speed received from roboclaw")
				print("failed")
			print("Right wheel speed:")
			if(self.speed_qppsr[0]):
				print(self.speed_qppsr[1])
			else:
				rospy.logwarn_throttle(1, "Invalid right speed received from roboclaw")
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to retrieve data.")

if __name__ == "__main__":
	rospy.init_node("roboclaw_node")
	rospy.loginfo("Initiating roboclaw node")
	rate = rospy.Rate(10)

	mdriver = MotorDriver()
	while not rospy.is_shutdown():
		#mdriver.publish_wheel_odom()
		rate.sleep()

