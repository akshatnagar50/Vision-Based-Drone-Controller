# Task 2 Submission

## Edits on presentation day
- 2 new files added `hover_presentation_day.py` and `rectangle_presentation_day.py`
- Files may look similar but they have a different `setpoint` element (1 setpoint is basically just hovering)
- On running 'hover_presentation_day.py', the drone hovers at a particular height and the disturbances in the horizontal plane are stabilised by the roll and pitch PID controllers. 
- On running `rectangle_presentaion_day.py` the drone uses PID to hover to a predefined height and then pass through four different setpoints thus forming a rectangular path.
- The videos of the same are included in this drive link: {https://drive.google.com/drive/folders/1-WnAKW1LL8b4X10ic06scJQUfW6rPd_-?usp=share_link}

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


## File Structure
PID.py
Runs the following Processes:
1. writeFunction     : Generates MSP packets and send it to the drone
2. readFunction     : Reads MSP packets from the drone and accesses the useful data
3. aruco_detect     : Takes the webcam feed and estimates the pose in camera frame of reference
4. PID                    : Generates userRC values based on the pose and setpoint 
5. key_handling     : Generates userRC values from keyboard
