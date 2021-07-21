#!/usr/bin/env python3

#import sys
#sys.path.insert(1, '/home/oceania/falcon_ws/src/falcon_drivers/scripts/ros_drivers/rpi_serial')
from roboclaw_3 import Roboclaw

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

# m1 = right motor
# m2 = left motor
# max linear speed  = 0.15m/s 
# max angular speed = 1.0rad/s

class MotorDriver(object):
	def __init__(self):
		self.is_open = False
		self.rc = Roboclaw("/dev/falcon_roboclaw" , 38400)
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
		self.RADIUS = 0.05
		self.WHEEL_BASE = 0.115+0.048
		self.MAX_LINEAR_SPEED = 0.13
		self.MAX_ANGULAR_SPEED = 0.63

		self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
		self.wheel_odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)

		self.odom_msg = Odometry()

		# setting to 0 
		self.w_qppsl = 0
		self.w_qppsr = 0
		
		self.last_time = rospy.get_rostime()
		self.current_time = rospy.get_rostime()
		self.x = 0
		self.y = 0
		self.z = 0
		self.th = 0
		self.speed_vx = 0
		self.speed_vy = 0
		self.speed_vz = 0

	def cmd_vel_cb(self, msg):
		self.v_x = msg.linear.x
		self.v_z = msg.angular.z 
		self.w_left = 0.5*(self.v_x - (self.v_z * self.WHEEL_BASE))/(self.RADIUS*self.PI)  #wheel left
		self.w_right = 0.5*(self.v_x + (self.v_z * self.WHEEL_BASE))/(self.RADIUS*self.PI) #wheel right 
		self.w_qppsl = (self.w_left * self.ENCODER_CPR * self.GEAR_RATIO)
		self.w_qppsr = (self.w_right * self.ENCODER_CPR * self.GEAR_RATIO)
		rospy.loginfo("Writing speeds to motors - Left: %2f, Right: %2f", self.w_left, self.w_right)
		rospy.loginfo("qpps to motors - Left: %d, Right: %d", int(self.w_qppsl), int(self.w_qppsr))
		self.write_speed("left", int(self.w_qppsl)) 
		self.write_speed("right", int(self.w_qppsr)) 
		# writing twice because sometimes it doesn't write properly
		self.write_speed("left", int(self.w_qppsl)) 
		self.write_speed("right", int(self.w_qppsr)) 

	def write_speed(self, motor, speed):
		if self.is_open:
			self.version = self.rc.ReadVersion(self.address)
			if motor == "right":
				self.rc.SpeedM1(self.address, speed)
			elif motor == "left":
				self.rc.SpeedM2(self.address, speed)
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to write data.")

	def publish_wheel_odom(self):
		if self.is_open:
			#self.read_encoder()
			self.read_speed()
			self.dt = (self.current_time - self.last_time).to_sec();
			self.dx = (self.speed_vx * math.cos(self.th) - self.speed_vy * math.sin(self.th)) * self.dt;
			self.dy = (self.speed_vx * math.sin(self.th) + self.speed_vy * math.cos(self.th)) * self.dt;
			self.dth = self.speed_vz * self.dt;

			self.x += self.dx;
			self.y += self.y;
			self.th += self.dth;
			self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
			self.odom_msg.pose.pose.position.x = self.x
			self.odom_msg.pose.pose.position.y = self.y
			self.odom_msg.pose.pose.position.z = self.z	# actually 0
			self.odom_msg.pose.pose.orientation.x = self.odom_quat[0]
			self.odom_msg.pose.pose.orientation.y = self.odom_quat[1]
			self.odom_msg.pose.pose.orientation.z = self.odom_quat[2]
			self.odom_msg.pose.pose.orientation.w = self.odom_quat[3]
			self.odom_msg.twist.twist.linear.x = self.speed_vx
			self.odom_msg.twist.twist.linear.y = self.speed_vy # actually 0
			self.odom_msg.twist.twist.angular.z = self.speed_vz
			self.wheel_odom_pub.publish(self.odom_msg)
			self.last_time = rospy.get_rostime()
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
			self.current_time = rospy.get_rostime()
			self.speed_qppsr = self.rc.ReadSpeedM1(self.address)
			self.speed_qppsl = self.rc.ReadSpeedM2(self.address)
			
			if(self.speed_qppsl[0]):
				print("Left wheel qpps:"+str(self.speed_qppsl[1])+";  Written qpps:"+str(self.w_qppsl))
				# convert from qpps to velocity
				self.speed_left = (self.speed_qppsl[1])/(self.ENCODER_CPR*self.GEAR_RATIO)
				print("Left wheel rps: " + str(self.speed_left))
			else:
				rospy.logwarn_throttle(1, "Invalid left speed received from roboclaw")
				# usually happens when the speed is not written properly. so, write the speed again
				#rospy.logwarn_throttle(0.2, "Attempting to write speed to left motor again")
				#self.write_speed("left", int(self.w_qppsl)) 
			if(self.speed_qppsr[0]):
				print("Right wheel qpps:"+str(self.speed_qppsr[1])+";  Written qpps:"+str(self.w_qppsr))
				# convert from qpps to velocity
				self.speed_right = (self.speed_qppsr[1])/(self.ENCODER_CPR*self.GEAR_RATIO)
				print("Right wheel rps: " + str(self.speed_right))
			else:
				rospy.logwarn_throttle(1, "Invalid right speed received from roboclaw")
				# usually happens when the speed is not written properly. so, write the speed again
				#rospy.logwarn_throttle(0.2, "Attempting to write speed to right motor again")
				#self.write_speed("right", int(self.w_qppsr)) 
			self.speed_vx = (self.speed_right + self.speed_left)*(2*self.PI)*(self.RADIUS/2)
			self.speed_vz = (self.speed_right - self.speed_left)*(2*self.PI)*(self.RADIUS/(2*self.WHEEL_BASE))
			print("Robot speed - x: " + str(self.speed_vx) + " z: " + str(self.speed_vz))
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to retrieve data.")

if __name__ == "__main__":
	rospy.init_node("roboclaw_node")
	rospy.loginfo("Initiating roboclaw node")
	rate = rospy.Rate(10)

	mdriver = MotorDriver()
	while not rospy.is_shutdown():
		mdriver.publish_wheel_odom()
		#mdriver.read_speed()
		rate.sleep()

