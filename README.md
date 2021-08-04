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

### Using Witmotion IMU:
Additional ECL library is required, can be installed using ``` sudo apt-get install ros-noetic-ecl-core ```
(Need to remove from falcon_drivers CMakeList.txt and package.xml and src/initial_orientation_wit.cpp if want to purge from workspace)
