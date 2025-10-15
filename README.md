# AURS

Hello. These are the scripts for my AURS Project. AURS, or Autonomous UAV for Rescue Signalling, is a drone that scans any given area for a specific target and then tracks + circles above the target to visually signal its location. It is designed for search and rescue missions. 

This project was simulated via ArduPilot SITL, Gazebo and ROS. It uses a YOLOv5 object detection model for target detection+tracking.

# Environment



To setup the environment, refer to the following guide for installing Gazebo 11, ROS NOETIC, Ardupilot SITL, and the Gazebo ROS plugin.

Refer to the same guide for setting a catkin_ws with your Gazebo Drone (Model: iris quadcopter with camera) Worlds located in it

Launch your world via Ros Command Line and then run the above scripts in a separate terminal.

Installation Guide: https://github.com/Intelligent-Quads/iq_tutorials/tree/master/docs        (By Intelligent Quads)
 
# Installations

For the scripts above, you will need the following python libraries:
dronekit 
rospy
pymavlink

threading
cv2
time
requests
random


