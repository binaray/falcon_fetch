#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
#import sys
#sys.path.insert(1, '/home/oceania/falcon_ws/src/falcon_drivers/scripts/ros_drivers/rpi_serial')
import adafruit_bno055

import rospy 
from sensor_msgs.msg import Imu

class ImuDriver(object):
	def __init__(self):
		self.is_open = False
		while(not self.is_open):
			try:
				self.i2c = board.I2C()
				self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
				self.last_val = 0xFFFF
				self.is_open = True
			except ValueError:
				rospy.logwarn("Imu i2c device with address 0x20 not found. Will attempt to reconnect in 1s")
				time.sleep(1)
		self.imu_pub = rospy.Publisher("bno055_imu/data", Imu, queue_size=10)
		self.imu_msg = Imu()

	def temperature(self):
		self.result = self.sensor.temperature
		if abs(self.result - self.last_val) == 128:
			self.result = self.sensor.temperature
			if abs(self.result - self.last_val) == 128:
				return 0b00111111 & self.result
		self.last_val = self.result
		return self.result

	def read_imu(self):
		if self.is_open:
			try:
				#print("Accelerometer (m/s^2): {}".format(self.sensor.acceleration))
				#print("Magnetometer (microteslas): {}".format(self.sensor.magnetic))
				#print("Gyroscope (rad/sec): {}".format(self.sensor.gyro))
				#print("Euler angle: {}".format(self.sensor.euler))
				print("Quaternion: {}".format(self.sensor.quaternion))
				#print("Linear acceleration (m/s^2): {}".format(self.sensor.linear_acceleration))
				#print("Gravity (m/s^2): {}".format(self.sensor.gravity))
				#print()
				self.imu_msg.orientation.w = self.sensor.quaternion[0]
				self.imu_msg.orientation.x = self.sensor.quaternion[1]
				self.imu_msg.orientation.y = self.sensor.quaternion[2]
				self.imu_msg.orientation.z = self.sensor.quaternion[3]
				self.imu_msg.angular_velocity.x = self.sensor.gyro[0]
				self.imu_msg.angular_velocity.y = self.sensor.gyro[1]
				self.imu_msg.angular_velocity.z = self.sensor.gyro[2]
				self.imu_msg.linear_acceleration.x = self.sensor.acceleration[0]
				self.imu_msg.linear_acceleration.y = self.sensor.acceleration[1]
				self.imu_msg.linear_acceleration.z = self.sensor.acceleration[2]
				self.imu_pub.publish(self.imu_msg)
			except OSError:
				rospy.logwarn("Imu cable is loose. Unable to obtain data")
				self.is_open = False
		else:
			# added delay
			rospy.logwarn("Attempting to reconnect to imu in 1s...")
			time.sleep(1)
			try:
				self.i2c = board.I2C()
				self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
				self.imu_msg.orientation.w = self.sensor.quaternion[0]
				self.imu_msg.orientation.x = self.sensor.quaternion[1]
				self.imu_msg.orientation.y = self.sensor.quaternion[2]
				self.imu_msg.orientation.z = self.sensor.quaternion[3]
				self.imu_msg.angular_velocity.x = self.sensor.gyro[0]
				self.imu_msg.angular_velocity.y = self.sensor.gyro[1]
				self.imu_msg.angular_velocity.z = self.sensor.gyro[2]
				self.imu_msg.linear_acceleration.x = self.sensor.acceleration[0]
				self.imu_msg.linear_acceleration.y = self.sensor.acceleration[1]
				self.imu_msg.linear_acceleration.z = self.sensor.acceleration[2]
				self.imu_pub.publish(self.imu_msg)
				self.is_open = True
			except OSError:
				rospy.logwarn("Imu is still disconnected.")
				self.is_open = False
			except ValueError:
				rospy.logwarn("Imu i2c device with address 0x20 not found.")

if __name__ == "__main__":
	rospy.init_node("bno055_node")
	rospy.loginfo("Initiating bno055 node")
	rate = rospy.Rate(10)

	idriver = ImuDriver()
	while not rospy.is_shutdown():
		idriver.read_imu()
		rate.sleep()
