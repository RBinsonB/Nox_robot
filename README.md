# Nox Robot project
Nox is a nice (and time-consuming) robot which uses SLAM (ROS) with a Kinect to navigate in its environment. It is powered by ROS running on an Raspberry Pi 3B and an Arduino that controls two motors with encoders.

# README IN PROGRESS - MORE EXPLANATION TO COME

# How to use Nox
This readme doesn't explain how to install ROS or the present package and assumes you already did so. For more information you can check the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials).

In order to navigate Nox you will need to start two launch files using SSH and [ROS nodes over network](http://wiki.ros.org/ROS/NetworkSetup).
### 1. Connect to Nox Wi-Fi
I haven't included in my GitHub the RPi network configuration but basically the RPi creates an Ad-Hoc network I connect to with my computer in order to send SSH commands. Once connected and before launching any ROS command you should sync the RPi to the computer using the following code in RPi:

`sudo ntpdate ip_address_of_the_computer`

You can also setup the ntpdate or use chrony to do that automatically.

### 2. Start the odometry and motor control
In the RPi, use the following command to start the main program:

`roslaunch nox nox_bringup.launch`

It will launch the serial controller node connected to the Arduino, the Kinect node, the odometry calculation node and the joint state publisher. If this step succeded you should see the side light of the robot going from a series of three quick blinks to a slow "breathing-like" type of blinking.

### 3. Start the navigation
In order to be able to do path planning calculation and mapping you have to use a launch file on your computer this time. Before doing so, assure yourself that you exported the ROS master from the RPi (modify the command to suit your own setup):

`export ROS_MASTER_URI=http://localhost:11311`

You can then launch:

`roslaunch nox nox_slam.launch`

It will launch the move_base node and RViz. By using the "2D Nav Goal" arrow you can give a goal for the robot to reach. Alternatively, you can use the ROS teleop keyboard to control the robot:

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py` 
