#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import sys
sys.path.insert(1, '/home/oceania/falcon_ws/src/falcon_drivers/scripts/ros_drivers/rpi_serial')
import adafruit_bno055

import rospy

class ImuDriver(object):
	def __init__(self):
		self.i2c = board.I2C()
		self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
		self.last_val = 0xFFFF

	def temperature(self):
		self.result = self.sensor.temperature
		if abs(self.result - self.last_val) == 128:
			self.result = self.sensor.temperature
			if abs(self.result - self.last_val) == 128:
				return 0b00111111 & self.result
		self.last_val = self.result
		return self.result

	def read_imu(self):
		print("Accelerometer (m/s^2): {}".format(self.sensor.acceleration))
		print("Magnetometer (microteslas): {}".format(self.sensor.magnetic))
		print("Gyroscope (rad/sec): {}".format(self.sensor.gyro))
		print("Euler angle: {}".format(self.sensor.euler))
		print("Quaternion: {}".format(self.sensor.quaternion))
		print("Linear acceleration (m/s^2): {}".format(self.sensor.linear_acceleration))
		print("Gravity (m/s^2): {}".format(self.sensor.gravity))
		print()

		time.sleep(1)

if __name__ == "__main__":
	rospy.init_node("bno055_node")
	rospy.loginfo("Initiating bno055 node")
	rate = rospy.Rate(10)

	idriver = ImuDriver()
	while not rospy.is_shutdown():
		idriver.read_imu()
		rate.sleep()
