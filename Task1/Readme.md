# Task 1 Submission
## Problem Statement for task 1

The first task required us to develop a python Wrapper for the Drone Pluto 1.2 using MSP packets and socket communication to control the basic movements of the drone (Arm, Take-off, Land, Throttle, Roll, Pitch, Yaw, etc). This was supposed to mirror the actions undertaken by the ROS package, but with direct python communication capable of being run on most machines without ROS or any supporting software. 
## Description
Our python wrapper is capable of running on both Windows and Linux machines and relies on simple python socket communication. We have provided a very basic teleop frontend for the drone, which can be used to simply fly the drone using keyboard keys. The same endpoints can also be used to make a controller drontend similar to the one in the Pluto mobile application. 
In order to execute the wrapper, the following steps can be followed. 
-Clone the Repository
-Navigate to the Task 1 folder
-Connect to the drone's wifi
-Run the PlutoArm.py file. 
-Control the drone using the keyboard commands
