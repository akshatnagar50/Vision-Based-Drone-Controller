# Team13
Official repository for the Inter IIT Tech aerial robotics PS

plutoPID.py
Runs the following Processes:
1. writeFunction     : Generates MSP packets and send it to the drone
2. readFunction     : Reads MSP packets from the drone and accesses the useful data
3. aruco_detect     : Takes the webcam feed and estimates the pose in camera frame of reference
4. PID                    : Generates userRC values based on the pose and setpoint 
5. key_handling     : Generates userRC values from keyboard
