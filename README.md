# complete_coverage
This is a collection of packages for the Robot Operating System (ROS). 

Together, these packages provide an implementation of an online complete coverage maneuvering system for unmanned surface vehicles (USVs).

## Installation
Navigate to the ```src/``` folder in your catkin workspace, e.g. ```cd ~/catkin_ws/src```. Then run the following (the command ```sudo rosdep init``` will print an error if you have already executed it since installing ROS. This error can be ignored.)
```
git clone https://github.com/jhlenes/complete_coverage.git --recurse-submodules
cd ..
sudo apt update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make
source devel/setup.bash
```

## Guidance
The guidance package implements a line-of-sight (LOS) guidance for path following of curved paths. 

The launch file sets up everything required for path following. It starts nodes for SLAM, map inflation, and LOS guidance. Furthermore, it sets up rviz for visualization. 
For simulation:
```
roslaunch guidance sim_guidance.launch
```
For actual USV:
```
roslaunch guidance real_guidance.launch
```
The USV is now ready start moving, all it needs is a path from one of the two complete coverage path planning methods.

## Complete coverage path planning
There are two different implementations of complete coverage. ```coverage_binn``` implements a bio-inspired neural network (BINN) approach. ```coverage_boustrophedon``` implements an approach based on boustrophedon (lawnmower pattern) motions. ```coverage_boustrophedon``` is currently the best performing method.  
```
roslaunch coverage_binn coverage_binn.launch
```
```
roslaunch coverage_boustrophedon coverage.launch
```
Once either of these are started while the ```guidance``` is running, the USV should start moving.

## Simulation
To simulate the USV, use the [usv_simulator](https://github.com/jhlenes/usv_simulator.git) package.


## Sensors
This system will connect to several sensors. In order to know which sensor is at which USB port (```/dev/ttyUSB*```), we need to set up some udev rules. These rules are defined for [RPLIDAR](etc/rplidar.rules), and [Xsens IMU](etc/xsens.rules). Apply them by:
```
sudo cp etc/rplidar.rules /etc/udev/rules.d
sudo cp etc/xsens.rules /etc/udev/rules.d
```
Unplug and replug your devices, and you are finished.

If you would like to add other sensors as well, you can find the ``` ATTRS{idVendor}``` and ```ATTRS{idProduct}``` with the ```lsusb``` command. If the names are cryptic, just unplug and replug your devices to see what is changing.

## Map inflating
The ```map_inflating``` package uses a [costmap](http://wiki.ros.org/costmap_2d) in order to inflate nearby obstacles.

## SLAM
The ```slam``` package performs simultaneous localization and mapping (SLAM) with [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/).

