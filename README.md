# SCARA_Motion_Planning
Motion Planning for SCARA Robot Arm, implemented in C++, visualized with Qt-GUI and Rviz.

## Summary
This work is my final project in the course Robotics, taught by Dr. Hoang-Giap Nguyen.

<p align="center">
  <img src="GUI.png" width="600" alt="accessibility text">
</p>
<p align="center">
  <img src="Rviz.png" width="600" alt="accessibility text">
</p>

## Build and Run
Use the following commands to download and compile the package (tested on Ubuntu 16.04 and 18.04)
```
git clone https://github.com/hapham258/SCARA_Motion_Planning.git
cd SCARA_Motion_Planning
catkin_make
source devel/setup.bash
roslaunch launch/start_model.launch
```
