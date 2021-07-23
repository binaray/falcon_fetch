#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# for GPIO pins
LEFT_WHEEL_PWM = 12
RIGHT_WHEEL_PWM = 13
LEFT_WHEEL_DIR = 6
RIGHT_WHEEL_DIR = 26
PWM_FREQ = 1000
FORWARD = 0 # inverted logic because using front wheel drive on rear wheel drive platform
BACKWARD = 1

#duty cycle 100 is stop
#duty cycle 0 is max speed
#duty cycle 70 is min speed. any higher is too slow

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # using board numbering
GPIO.setup(RIGHT_WHEEL_DIR,GPIO.OUT)
GPIO.setup(LEFT_WHEEL_DIR,GPIO.OUT)
GPIO.setup(RIGHT_WHEEL_PWM,GPIO.OUT)
GPIO.setup(LEFT_WHEEL_PWM,GPIO.OUT)
right_wheel = GPIO.PWM(RIGHT_WHEEL_PWM,PWM_FREQ)
left_wheel = GPIO.PWM(LEFT_WHEEL_PWM,PWM_FREQ) 
right_wheel.start(0)
left_wheel.start(0)


while True:
	try:
		GPIO.output(RIGHT_WHEEL_DIR, FORWARD)
		GPIO.output(LEFT_WHEEL_DIR, BACKWARD) # motor is inverted
		print("forward")
		time.sleep(3)
		GPIO.output(RIGHT_WHEEL_DIR, BACKWARD)
		GPIO.output(LEFT_WHEEL_DIR, FORWARD)
		print("backward")
		time.sleep(3)
		GPIO.output(RIGHT_WHEEL_DIR, FORWARD)
		GPIO.output(LEFT_WHEEL_DIR, FORWARD)
		print("left")
		time.sleep(3)
		GPIO.output(RIGHT_WHEEL_DIR, BACKWARD)
		GPIO.output(LEFT_WHEEL_DIR, BACKWARD)
		print("right")
		time.sleep(3)
		GPIO.output(RIGHT_WHEEL_DIR, FORWARD)
		GPIO.output(LEFT_WHEEL_DIR, BACKWARD)
		right_wheel.ChangeDutyCycle(0)
		left_wheel.ChangeDutyCycle(0)
		print("changing speed. starting with duty cycle: 0")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(10)
		left_wheel.ChangeDutyCycle(10)
		print("duty cycle: 10")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(20)
		left_wheel.ChangeDutyCycle(20)
		print("duty cycle: 20")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(30)
		left_wheel.ChangeDutyCycle(30)
		print("duty cycle: 30")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(40)
		left_wheel.ChangeDutyCycle(40)
		print("duty cycle: 40")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(50)
		left_wheel.ChangeDutyCycle(50)
		print("duty cycle: 50")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(60)
		left_wheel.ChangeDutyCycle(60)
		print("duty cycle: 60")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(70)
		left_wheel.ChangeDutyCycle(70)
		print("duty cycle: 70")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(80)
		left_wheel.ChangeDutyCycle(80)
		print("duty cycle: 80")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(90)
		left_wheel.ChangeDutyCycle(90)
		print("duty cycle: 90")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(100)
		left_wheel.ChangeDutyCycle(100)
		print("duty cycle: 100")
		time.sleep(3)
		right_wheel.ChangeDutyCycle(50)
		left_wheel.ChangeDutyCycle(50)
	except KeyboardInterrupt:
		#right_wheel.ChangeDutyCycle(0)
		#left_wheel.ChangeDutyCycle(0)
		GPIO.cleanup()
