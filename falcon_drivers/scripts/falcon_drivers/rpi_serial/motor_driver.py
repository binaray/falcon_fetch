#!/usr/bin/env python3

#import sys
#sys.path.insert(1, '/home/oceania/falcon_ws/src/falcon_drivers/scripts/ros_drivers/rpi_serial')
from roboclaw_3 import Roboclaw

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# m1 = right motor
# m2 = left motor
# for the falcon test platform, m1 and m2 encoders are swapped. actual platform should not be swapped

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
		self.L = 1.6 # wheel base
		self.R = 0.4 # Radius of the wheel

		self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
		self.wheel_odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)

		self.odom_msg = Odometry()

		# setting to 0 
		self.w_qppsl = 0;
		self.w_qppsr = 0;
		self.is_turning = 0;
		self.is_reverse = 0;

	def cmd_vel_cb(self, msg):
		self.v_x = msg.linear.x
		self.v_z = msg.angular.z
		# the following flags are because the motor sucks and cannot move back at the same speed
		if self.v_z != 0:
			self.is_turning = True
		else:
			self.is_turning = False
		if self.v_x < 0:
			self.is_reverse = True
		else:
			self.is_reverse = False
		self.w_left = ((2 * self.v_x) - (self.v_z * self.L)) / (2 * self.R)  #wheel left
		self.w_right = ((2 * self.v_x) + (self.v_z * self.L)) / (2 * self.R) #wheel right 
		self.w_qppsl = (self.w_left * self.ENCODER_CPR * self.GEAR_RATIO)/(2 * self.PI)
		self.w_qppsr = (self.w_right * self.ENCODER_CPR * self.GEAR_RATIO)/(2 * self.PI)
		rospy.loginfo("Writing speeds to motors - Left: %2f, Right: %2f", self.w_left, self.w_right)
		rospy.loginfo("qpps to motors - Left: %d, Right: %d", int(self.w_qppsl), int(self.w_qppsr))
		self.write_speed("left", int(self.w_qppsl)) 
		self.write_speed("right", int(self.w_qppsr)) 

	def write_speed(self, motor, speed):
		if self.is_open:
			self.version = self.rc.ReadVersion(self.address)
			# convert to pwm mode
			if abs(speed) > 3700:
				pwm_speed = 128 # max speed
			elif abs(speed) == 0:
				pwm_speed = 0 # 0 speed
			else:
				pwm_speed = int(speed/3700*128)
			
			if motor == "right":
				#self.rc.SpeedM1(self.address, pwm_speed)
				if speed > 0:
					self.rc.ForwardM1(self.address, pwm_speed)
				else:
					self.rc.BackwardM1(self.address, abs(pwm_speed))
			elif motor == "left":
				#self.rc.SpeedM2(self.address, pwm_speed)
				# m2 motor spoil so need to limit max speed for robot to turn properly
				if self.is_reverse or self.is_turning:
					if abs(speed) > 1600:
						pwm_speed = 64 # max speed
					elif abs(speed) == 0:
						pwm_speed = 0 # 0 speed
					else:
						pwm_speed = int(speed/1600*64)
				if speed > 0:
					self.rc.ForwardM2(self.address, pwm_speed)
				else:
					self.rc.BackwardM2(self.address, abs(pwm_speed))
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
			# logic swapped because encoder wrong
			self.speed_qppsl = self.rc.ReadSpeedM1(self.address)
			self.speed_qppsr = self.rc.ReadSpeedM2(self.address)
			
			if(self.speed_qppsl[0]):
				print("Left wheel qpps:"+str(self.speed_qppsl[1]))
				# convert from qpps to velocity
				self.speed_left = (self.speed_qppsl[1])/(self.ENCODER_CPR*self.GEAR_RATIO)
				print("Left wheel rps: " + str(self.speed_left))
			else:
				rospy.logwarn_throttle(1, "Invalid left speed received from roboclaw")
			if(self.speed_qppsr[0]):
				print("Right wheel qpps:"+str(self.speed_qppsr[1]))
				# convert from qpps to velocity
				self.speed_right = (self.speed_qppsr[1])/(self.ENCODER_CPR*self.GEAR_RATIO)
				print("Right wheel rps: " + str(self.speed_right))
			else:
				rospy.logwarn_throttle(1, "Invalid right speed received from roboclaw")
			self.speed_vx = self.R*(self.speed_right + self.speed_left)/2
			#self.speed_vx = self.R*2*self.PI*(self.speed_right+self.speed_left)/60
			self.speed_vy = 0
			self.speed_vz = self.R*(self.speed_right - self.speed_left)/(2*self.L)
			print("Robot speed - x: " + str(self.speed_vx) + " z: " + str(self.speed_vz))
		else:
			rospy.logwarn_throttle(1, "Motor driver is not connected. Unable to retrieve data.")

if __name__ == "__main__":
	rospy.init_node("roboclaw_node")
	rospy.loginfo("Initiating roboclaw node")
	rate = rospy.Rate(10)

	mdriver = MotorDriver()
	while not rospy.is_shutdown():
		#mdriver.publish_wheel_odom()
		mdriver.read_speed()
		rate.sleep()

