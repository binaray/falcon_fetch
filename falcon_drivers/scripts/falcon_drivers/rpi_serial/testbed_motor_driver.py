#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
import rospy
import math
from geometry_msgs.msg import Twist

FORWARD = 0 # inverted logic because using front wheel drive on a read wheel drive platform
BACKWARD = 1

# max linear speed ~0.09426m/s
# max angular speed ~0.71409rad/s
# max rpm ~30rev/s

# duty cycle 100 is stop
# duty cycle 0 is max speed
# duty cycle 70 is min speed. any higher is too slow

class MotorDriver(object):
	def __init__(self):
		self.RADIUS = 0.03
		self.WHEEL_BASE = 0.087+0.045
	
		# for GPIO pins
		self.LEFT_WHEEL_PWM = 12
		self.RIGHT_WHEEL_PWM = 13
		self.LEFT_WHEEL_DIR = 6
		self.RIGHT_WHEEL_DIR = 26
		self.PWM_FREQ = 1000
		self.MAX_RPM = 30

		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM) # using board numbering
		GPIO.setup(self.RIGHT_WHEEL_DIR,GPIO.OUT)
		GPIO.setup(self.LEFT_WHEEL_DIR,GPIO.OUT)
		GPIO.setup(self.RIGHT_WHEEL_PWM,GPIO.OUT)
		GPIO.setup(self.LEFT_WHEEL_PWM,GPIO.OUT)
		self.right_wheel = GPIO.PWM(self.RIGHT_WHEEL_PWM,self.PWM_FREQ)
		self.left_wheel = GPIO.PWM(self.LEFT_WHEEL_PWM,self.PWM_FREQ) 
		self.right_wheel.start(100)
		self.left_wheel.start(100)
		
		self.prev_w_left = 0
		self.prev_w_right = 0
		
		self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)
		
	def cmd_vel_cb(self, msg):
		self.v_x = msg.linear.x
		self.v_z = msg.angular.z 
		self.w_left = 0.5*(self.v_x - (self.v_z * self.WHEEL_BASE))/(self.RADIUS*math.pi)*60  # wheel left in rpm
		self.w_right = 0.5*(self.v_x + (self.v_z * self.WHEEL_BASE))/(self.RADIUS*math.pi)*60 # wheel right in rpm
		rospy.loginfo("Writing speeds(rpm) to motors - Left: %2f, Right: %2f", self.w_left, self.w_right)
		if(self.w_left!=self.prev_w_left):
			self.write_speed("left", self.w_left)
			self.prev_w_left = self.w_left # to prevent continously writing and spamming the motor
		if(self.w_right!=self.prev_w_right):
			self.write_speed("right", self.w_right)
			self.prev_w_right = self.w_right
		
	def write_speed(self, wheel, speed):
		if speed > self.MAX_RPM:
			speed = self.MAX_RPM # keep max at 30
		pwm_speed = int((self.MAX_RPM-abs(speed))/self.MAX_RPM*100) # inverted duty cycle logic for some reason
		if wheel=="left":
			if speed >= 0:
				GPIO.output(self.LEFT_WHEEL_DIR, BACKWARD)
			else:
				GPIO.output(self.LEFT_WHEEL_DIR, FORWARD)
			self.left_wheel.ChangeDutyCycle(pwm_speed)
			#print("writing pwm speed:" + str(pwm_speed))
		if wheel=="right":
			if speed >= 0:
				GPIO.output(self.RIGHT_WHEEL_DIR, FORWARD)
			else:
				GPIO.output(self.RIGHT_WHEEL_DIR, BACKWARD)
			self.right_wheel.ChangeDutyCycle(pwm_speed)
			#print("writing pwm speed:" + str(pwm_speed))
			
if __name__ == "__main__":
	rospy.init_node("falcon_motor_driver_node")
	rospy.loginfo("Initiating falcon motor driver node")
	rate = rospy.Rate(10)

	mdriver = MotorDriver()
	while not rospy.is_shutdown():
		rate.sleep()
	#stop the robot if ros dies
	GPIO.cleanup()
