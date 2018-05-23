# Multimodal Writer

The goal of this package is to provide a handwriting recorder that records handwriting traces on a tablet as well as hand motion with a leap motion. 

(native support of ROS)

Currently only gets the finger tips. 

Standalone
------------
## Installation
TODO


ROS Support
------------
### Installation
```
$ cd ~/catkin_ws 
$ catkin_make
$ source devel/setup.sh
```

### Usage
Once compiled with catkin, you can launch the multimodal_writer package
```
$ roslaunch multimodal_writer listener_launch.launch
```
The tf messages are boradcasted when hand are detected with the leap motion. 
You can visualize the hand by using the rviz config provided in the launch directory.

