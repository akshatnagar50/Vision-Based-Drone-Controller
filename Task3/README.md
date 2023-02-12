# Task 3

## Description
- Task 3 expects one drone to make a rectangle and the second drone to follow the drone with a lag of one setpoint.

## Code Explanation
- plutoMultiWii_with_swarm.py has the functions to communicate with the swarm of drones. It creates and sends MSP packets to the appropriate drone based on the commands given from swarm_mission.py
- swarm_mission.py generates the commands to be given to the drones based on Task 3 description using a high level path planning algorithm and low level PID controller.
   
## Steps to Connect

* Connect to the Pluto WiFi 
* `telnet 192.168.4.1`
* `+++AT MODE 3`
* `+++AT STA [SSID] [password]`
* Repeat the same for second drone.
* Connect laptop to the same WiFi
