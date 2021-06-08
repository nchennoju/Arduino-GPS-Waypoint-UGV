# Arduino-GPS-Waypoint-UGV

The following project is a completely custom Arduino based project designed to be the first step in designing a Flight Controller with GPS capabilities. The following flight controller uses two Arduinos, 1 BN880 GPS module, 1 MPU9250 9 axis IMU, 1 NRF24 radio module, and a analog voltage sensor. More sensors will be interfaced as the flight controller approaches flight readiness, but until then, this UGV platform is more than plenty to test flight controller features. At the moment, the UGV is capable of running GPS waypoint missions with a 1M accuracy while being able to navigate at a top speed of 10mph (a fast jog). Below is an image of the UGV...

![UGV](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Images/IMG_3642.jpg)


To simplify wiring, a custom PCB was designed. The PCB has been oversized at the moment as fligth controller mass and size are nearly as important when equipped on a ground vehicle. For now, V1 of the flight controller PCB offers more than enough space and flexibility, but will eventually be redesigned.


![PCB](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Images/Screenshot%202021-05-14%20115919.jpg)


Additionally, telemetry capabilities were added to the rover for better ease in debugging while also offering users a user friendly interface to monitor the status of the UGV. The GUI dashboard was designed in python keeping in mind that the following hardware will eventually be equipped on an Aerial device. Using a recieving NRF24 radio module connected to an arduino to serialize inputs, the python GUI uses the pyserial library to extract UGV data and graphically represents it. Currently, attitude, GPS position, voltage, heading data, and PID inputs are shown.


![GUI](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Images/2021-06-06.png)
