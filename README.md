# RobotCompetition - Rob-bottle

## General
This code was written for the robot competition 2022 at EPFL (https://robot-competition.epfl.ch/). It was used for Rob-bottle (our robot) that collects pet bottles 
and brings them back to a recycling area. The contributors are Brodier Mariane, Ourion Cyril and Schmid Nicolaj.

## Arduino
In this repository there are all files used for the arduino. The arduino does the low level computation: motor control (wheel, arm and basket motors) and the 
ultrasonic sensor reading. The file flashed on the arduino is called _motor_sensor_arm_controller_.

## catkin_ws_raspberry
This repository contains the source files of the _catkin workspace_ run on a _Raspberry Pi 4_. It is a _ROS noetic_ implementation and does the high level computation.
The following packages are used:
* __bb_mpu9150__: integration of IMU _mpu9150_ (see https://github.com/vmayoral/bb_mpu9150). The IMU was not used in the final version.
* __hector_slam__: SLAM algorithm (see http://wiki.ros.org/hector_slam).
* __robot__: custom package that does all of the work (sensor integration, measurement processing, path planning, decision making, ...).
* __rosserial__: library used for the windows node (see _visualization_windows_).
* __rplidar_ros__: integration of the _Slamtec A1M8_ LIDAR (see https://www.slamtec.com/en/Lidar/A1).

## visualization_windos
The files in this repositroy can be run on a windows machine and are visualizing the calculations:
* __path_planner_visualization__: contains a pythons script that plots the robot's trajectory.
* __real_time_map_visualization__: is a _ros node_ for _Visual Studio_ and shows the map of the robot's surrounding.

## Launching Rob-bottle
Assuming that the _catkin workspace_ and _ROS noetic_ are installed correctly on the _Raspberry Pi_, the code can be compiled as follows:

&nbsp; &nbsp; &nbsp; &nbsp; _cd catkin_ws_

&nbsp; &nbsp; &nbsp; &nbsp; _catkin_make_

Then, _motor_sensor_arm_controller_ must be flashed on the arduino. And finally, the robot can be launched with the launch file _robot.launch_ from the package _robot_:

&nbsp; &nbsp; &nbsp; &nbsp; _roslaunch robot robot.launch_
