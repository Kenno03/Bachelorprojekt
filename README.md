# Bachelorprojekt
This is the GitHub which contains all the data, video, MatLab Scripts, and C-code which were used in the Bachelor projekt 'Convoy Control of Mobile Robots' by Markus Kenno Hansen (8th of June 2025).
## Author
Markus Kenno Hansen

## Table of Contents
- [Overview](#overview)
- [How to run follower_robot.c](#how-to-run-follower_robotc)
- [Good to know](#good-to-know)


## Overview
- The Matlab scripts were used for data-analysis.
- The different C programs are either ment for data collection, or as to control the SMR-robots.

### How to run follower_robot.c
I personally used VS-code with the ssh - extension, which allows you to ssh into a pc in the VS-code environment, which made it easy to run and execute code.
- Go get an SMR robot, turn it on, and wait for a bit.
- Open Visual Studio Code and make sure you have the ssh extension.
  - Navigate to the 'live' folder in each robot and insert the follower_robot.c program and the json files.
- For each follower robot open two command prompts / terminals.
  - type "ssh k388hX@smrN.iau.dtu.dk" hit 'enter' and type the password; where X is the team name (e.g. 1) and N is the SMR Id, which is found as a label on each robot.
  - When logged in for each terminal type 'cd live'.
    - In one terminal type 'mrc -tX'to open a telnet connection. X is will be the IP of telnet conenction (3100X) and it is imporrtant to note, as the follower program needs this. This is where the commands are being set.
    - In the other terminal type 'ulmsserver' and hit enter to start the LiDAR server.
- Edit the lines
   ```c
   #define SERVER_IP "192.38.66.88" //ip "192.38.66.80+X, X = smrX"
   #define CMD_PORT 31008  //mrc 3100X -tx
where X has to be the same number as for the SMR robot. in this example X = 8 for both, though they can be different.
 - With the telnet connection open and the ulmmsserver running, AND the robot placed behind another robot, run the follwer_robot.C program in VS code. ** ⚠️NOTE ** when running the program, be ready to either close the telnet connection from the terminal, or type 'stop' in the VS code terminal, in case the robot should be about to colide. 
Do this for each robot you want in the convoy.

### Good to know
To extract the data files after each run, in the VS code environment, navigate to the log files, right click, and select "download".
📌 If the conenctions are established correctly, in the VS code terminal, the program will print the initial coordinates of the robot (local coordinates). If not, it will state which connection were not estabished.
⚠️ Microsoft issued an update that made the SSH extension not work with some linux version. This can be a problem if the Linux version is not updated. Any VS version pre March 2025 should work, just make sure to turn off auto updates.

To run any of the MATLAB Scripts, rename the file-paths to yours.
