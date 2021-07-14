# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#!/usr/bin/env python
import time
import board
import adafruit_bno055
import neopixel
#import VL53L0X
import RPi.GPIO as GPIO
from gpiozero import Button
import roboclaw
from roboclaw_3 import Roboclaw
rc = Roboclaw("/dev/ttyACM0" , 38400)
rc.Open()

# GPIO for Sensor 1 shutdown pin
#sensor1_shutdown = 20
# GPIO for Sensor 2 shutdown pin
#sensor2_shutdown = 16
#message = ''
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup GPIO for shutdown pins on each VL53L0X
#GPIO.setup(sensor1_shutdown, GPIO.OUT)
#GPIO.setup(sensor2_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
#GPIO.output(sensor1_shutdown, GPIO.LOW)
#GPIO.output(sensor2_shutdown, GPIO.LOW)

# Create one object per VL53L0X passing the address to give to
# each.
#tof = VL53L0X.VL53L0X(address=0x2B)
#tof1 = VL53L0X.VL53L0X(address=0x2D)

# Set shutdown pin high for the first VL53L0X then 
# call to start ranging 
#GPIO.output(sensor1_shutdown, GPIO.HIGH)
#time.sleep(0.50)
#tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# Set shutdown pin high for the second VL53L0X then 
# call to start ranging 
#GPIO.output(sensor2_shutdown, GPIO.HIGH)
#time.sleep(0.50)
#tof1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

#timing = tof.get_timing()
#if (timing < 20000):
 #   timing = 20000
#print ("Timing %d ms" % (timing/1000))


i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)
pixel_pin = board.D21
num_pixels = 8

angle = 45
angle1 = 90
Vq = 0
w = 0
address = 0x80
#imu
last_val = 0xFFFF 

#neopixel
ORDER = neopixel.GRB # led to grb order

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)
# neopixel
pixels.fill((100, 100, 0))
pixels.show()

def on_message(client, userdata, msg):
    global message
    #print(msg.topic+" "+str(msg.payload))
    topic = str(msg.topic)
    if(topic == "fetch/motion"):
        locomotion = str(msg.payload.decode("utf-8"))
        print(locomotion)
        if(locomotion == "f"):
            Vq = 20
            w = 0
        elif(locomotion == "b"):
            Vq = -20
            w = 0
        elif(locomotion == "r"):
            Vq = 0
            w = 15
        elif(locomotion == "l"):
            Vq = 0
            w = -15
        elif(locomotion == "s"):
            Vq = 0
            w = 0
        else:
            print("invalid")
        ENCODER_CPR = 20  #encoder CPR
        GEAR_RATIO = 391 # Gear-ratio
        PI = 3.14 
        L = 1.6 # wheel base
        R = 0.4 # Radius of the wheel
        w_right = ((2 * Vq) + (w * L)) / (2 * R) #wheel right 
        w_left = ((2 * Vq) - (w * L)) / (2 * R)  #wheel left

        w_qppsl = (w_left * ENCODER_CPR * GEAR_RATIO)/(2 * PI)
        w_qppsr = (w_right * ENCODER_CPR * GEAR_RATIO)/(2 * PI)

        w1 = round(w_qppsl, 2)
        w2 = round(w_qppsr, 2)
        w3 = int(w1) #wheel right 
        w4 = int(w2) #wheel left

        global address 
        version = rc.ReadVersion(address)
        rc.SpeedM1(address, w3)
        rc.SpeedM2(address, w4)
    #displayspeed()
    
    #imu euler angle is degrees, first value corresponds to yaw
    
    #elif(topic == "fetch/sleep"):
     #   sleep = str(msg.payload.decode("utf-8"))
      #  print(sleep)
       # if(sleep == "h"):
        #    GPIO.output(24, False) 
         #   time.sleep(10)
          #  GPIO.output(24, True) 
        #elif(sleep == "w"):
         #   GPIO.output(24, True) 
    

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
while True:
    print("loop")
    print("Euler angle: {}".format(sensor.euler))
    # tof1 distance 
    #distance = tof.get_distance()
    #tof2 distance
    #distance1 = tof1.get_distance()
    #if(distance >= 120 or distance1 >= 120):
        #print("stop")
        #rc.SpeedM1(address, 0)
        #rc.SpeedM2(address, 0)    
   
#tof1.stop_ranging()
#GPIO.output(sensor2_shutdown, GPIO.LOW)
#tof.stop_ranging()
#GPIO.output(sensor1_shutdown, GPIO.LOW)

