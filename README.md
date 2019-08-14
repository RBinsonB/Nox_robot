# Nox Robot project
Nox is a nice (and time-consuming) robot which uses SLAM (ROS) with a Kinect to navigate in its environment. It is powered by ROS running on a Raspberry Pi 3B and an Arduino that controls two motors with encoders.

In its current state the robot can use SLAM (gmapping) to create a map of its surroundings (using the Kinect depth perception to detect wall and obstacles) and localize itself within the map. It can plan a path to a given goal and drive to it avoiding obstacles.

## README IN PROGRESS - MORE EXPLANATION TO COME
Information can be found in the [Hackster page of the project](https://www.hackster.io/robinb/nox-a-house-wandering-robot-ros-652315).

## How to use Nox
### Packages installation
The software for the Nox project was developped with ROS Kinetic and Ubuntu 16.04. More recent versions should work as well but might require some tweaking. This readme doesn't explain how to install ROS and packages. For more information you can check the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials).

#### To use Nox you will need the following packages (most of them should already be installed by default or requested when building the nox packages):
- The [navigation stack](https://wiki.ros.org/navigation),
- The freenect package (for connecting to the Kinect)
- [RViz](http://wiki.ros.org/rviz)
- [TF](http://wiki.ros.org/tf), [Joint State](http://wiki.ros.org/joint_state_publisher) and [Robot State](http://wiki.ros.org/robot_state_publisher) Publishers
- [ROSSerial package](http://wiki.ros.org/rosserial) (for connecting to the Arduino Mega)
- [SLAM gmapping](https://wiki.ros.org/slam_gmapping).

#### Installation
You can install and build the package by copying the "nox" and "nox_description" folders in "<your catkin workspace>/src" and running:
  ```
  catkin_make
  ```

## Running Nox

In order to navigate Nox you will need to start two launch files using SSH and [ROS nodes over network](http://wiki.ros.org/ROS/NetworkSetup).
### 1. Connect to Nox Wi-Fi
I haven't included in my GitHub the RPi network configuration but basically the RPi creates an Ad-Hoc network I connect to with my computer in order to send SSH commands. It's also possible to connect both the Rpi and the computer to the same WiFi network. Once connected and before launching any ROS command you should sync the RPi to the computer using the following code in RPi:

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
