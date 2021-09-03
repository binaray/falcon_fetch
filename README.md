# falcon_fetch

Repository for falcon fetch

## Setup for RPI
1. Create i2c and gpio groups using ``` sudo addgroup i2c && sudo addgroup gpio ```
2. Add user into both groups using ``` sudo adduser ${USERNAME} i2c && sudo adduser ${USERNAME} gpio ```
3. Copy the 72-falcon-i2c.rules into /etc/udev/rules.d by ``` sudo cp ${WORKSPACE_DIR}/falcon_fetch/setup_files/72-falcon-i2c.rules /etc/udev/rules.d ```
4. Add the following line into /boot/firmware/usercfg.txt ``` dtparam=i2c_arm=on,i2c_arm_baudrate=50000 ```
5. Install adafruit bno055 library using ``` sudo pip3 install adafruit-circuitpython-bno055 ```
(More info found at https://github.com/adafruit/Adafruit_CircuitPython_BNO055)

### If using PiCam:
Add the following lines into /boot/firmware/config.txt 
``` start_x = 1 ```
``` gpu_mem = 128 ```
Video device shows up as /dev/video* after that.
To verify the connection, use ``` qv4l2 ``` to check if the camera is connected and able to view image stream from there.

** For RPI HQ Cam and running opencv2 with python script, second argument for VideoCapture is required
```
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
```

### Using Witmotion IMU:
Additional ECL library is required, can be installed using ``` sudo apt-get install ros-noetic-ecl-core ```
(Need to remove from falcon_drivers CMakeList.txt and package.xml and src/initial_orientation_wit.cpp if want to purge from workspace)

## To Run:
Roslaunch falcon_prog main.launch. This launches the following nodes. 
- falcon_prog- main program handling the robot states 
- marvelmind_nav- marvelmind beacon interface
- falcon_orientation- orientation callibration service node (using witmotion imu)
- wit_node: witmotion imu
-----
- To launch the motor drivers on falcon testbed platform (integrated motor driver): ``` rosrun falcon_drivers testbed_motor_driver.py ```
- To launch the motor drivers on falcon fetch platform (roboclaw): ``` rosrun falcon_drivers motor_driver_speed.py ```
- To launch the bno055 IMU on falcon fetch platform: ``` rosrun falcon_drivers imu.py ```
- To launch the camera (ros usb_cam package): ``` rosrun falcon_cam usb_cam.launch ```
- To launch the camera (opencv2 python script): ``` rosrun falcon_cam camera.py ```

```
# FALCON PROG PARAMS:
- stationary_beacon_count 
Number of stationary beacons the program will wait for response before executing the program

- x_step, bound_padding
Unused test params for dynamic waypoint generation based on beacon location

- max_linear_speed, max_angular_speed, min_linear_speed, min_angular_speed
Min and max speed for angular and linear movement of the robot. 
*Note control function of the robot does not handle both together accurately yet

- inflation_radius
Distance of robot to a point before linear speed of the robot decreases

- rotation_falloff
Rotation range of robot towards a point before robot angular speed decreases

- kP, kI
Unused params for test PID control

- stationary_threshold
Min distance jitter used to determine if the robot is stationary

- rotation_threshold
Rotation range of robot to move goal robot must adhere to before it can consider linear movement

- distance_threshold
Max distance of robot to move goal to determine goal reached

- waypoint_filepath
Waypoint file to load waypoints
```
