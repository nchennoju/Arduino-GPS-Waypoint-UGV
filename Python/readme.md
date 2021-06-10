# Python Files: programming GPS waypoints + telemetry GUI

The following steps should be followed is you would like to automate the process of enterring coordinates into the program.

Step 1) Download iNav: there should be many tutorial online to do this
Step 2) Open iNav, Navigate to Mission Control, Draw your Mission, and Save file
![Sample Mission](https://github.com/nchennoju/Arduino-GPS-Waypoint-UGV/blob/master/Python/2021-06-10.png)
Step 3) Open GPS_MissionPlanner.py and edit the line starting with "GPS_PROGRAM_PATH = ...". Enter the file location of the GPS_i2c_Slave cpp file.
Step 4) Run the python script, select the .mission file previously saved, and the GPS_i2c_Slave main.cpp file shoud be populated with the coordinates
Step 5) Upload program onto GPS Arduino
