# Python Files: programming GPS waypoints + telemetry GUI

The following steps should be followed is you would like to automate the process of enterring coordinates into the program.

Step 1) Download iNav: there should be many tutorial online to do this
Step 2) Open iNav, Navigate to Mission Control, Draw your Mission, and Save file
![Sample Mission](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Python/2021-06-10.png)

The following project is a completely custom Arduino based project designed to be the first step in designing a Flight Controller with GPS capabilities. The following flight controller uses two Arduinos, 1 BN880 GPS module, 1 MPU9250 9 axis IMU, 1 NRF24 radio module, and a analog voltage sensor. More sensors will be interfaced as the flight controller approaches flight readiness, but until then, this UGV platform is more than plenty to test flight controller features. At the moment, the UGV is capable of running GPS waypoint missions with a 1M accuracy while being able to navigate at a top speed of 10mph (a fast jog). Below is an image of the UGV...

![UGV](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Images/IMG_3642.jpg)

Download iNav
