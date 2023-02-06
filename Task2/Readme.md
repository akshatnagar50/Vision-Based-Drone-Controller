# Task 2 Submission
## Problem Statement for Task 2
The problem statement for the second task requires us to develop another layer of control over the drone than the PID that is built into the drone (To control the motors using RC commands). 
This is to be achieved with the help of an aruco marker placed on top of the drone and a ceiling mounted camera. Using this, we need to hover the drone in one position and then make it move in 
a (1x2)m rectangle shape. 
## Description
Our PID.py file is built on top of the python wrapper used in the previous code. The overhead camera uses the coordinates obtained by the center of the detected aruco marker, scaled to its size to 
send control commands to the respective pitch and roll directions. We make use of the Yaw from the magnetometer of the drone to guide the correct pitch and roll to x and y coordinate systems. 
The new values are finally sent as MSP packets to the drone.  
To run the file, we need to do the following:
- Clone the repository and navigate to the Task 2 folder
- Create the arena setup in the same way as described in the PS. 
- Connect to the drone wifi
- Run the PID.py code
- Arm the drone using spacebar
- Run the visual PID using the F key
## Videos
The hardware was giving us issues in providing for consistent throttle values, as explained and demonstrated in the video https://drive.google.com/file/d/1FYqVB6VO7ASdi389K2a-VSWthR2lI8Hi/view?usp=sharing
Hence, we made use of a tethered setup to demonstrate the working of our PID. You can see the roll and pitch corrections in the video https://drive.google.com/file/d/1d8S6eVZEuDbPjanBHCWFgLln-T9JxUa6/view?usp=share_link as well as their screen views https://drive.google.com/file/d/1CgVmbV8r6yZjJ937asmtzSBSgySwIxhS/view?usp=share_link and https://drive.google.com/file/d/1iN3c9rZqiCp0yz-l7tLtP2NruTPFqu24/view?usp=share_link  
The videos verify the algorithmic correctness of the controller. The precise PID gain tuning is somewhat rough as it was difficult to achieve without consistent throttle values and altitude. 
