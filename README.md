# complete_coverage
This is a collection of packages for the Robot Operating System (ROS). Works on Noetic and Melodic. 

Together, these packages provide an implementation of an online complete coverage maneuvering system for unmanned surface vehicles (USVs).

Details can be found in my Master's thesis:

Lenes, Jan Henrik. "Autonomous online path planning and path-following control for complete coverage maneuvering of a USV." Master's thesis, NTNU, 2019.
http://hdl.handle.net/11250/2622919

See the code in action at: https://www.youtube.com/watch?v=hqOUKtosnFw

## Installation

### Install Cartographer
Cartographer is no longer available through rosdep, and must be built from source.

Follow the instructions at: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#

I recommend to install Cartographer in its own workspace, for example `~/cartographer_ws`. Cartographer is huge and only needs to be built once, so keeping it separate makes things a little simpler.

### Installing
Navigate to the ```src/``` folder in your catkin workspace, e.g. ```cd ~/catkin_ws/src```. Then run the following (the command ```sudo rosdep init``` will print an error if you have already executed it since installing ROS. This error can be ignored.)
```
git clone https://github.com/jhlenes/complete_coverage.git --recurse-submodules

cd ..

# This is not available through rosdep, so we install manually
sudo apt update
sudo apt install python3-serial

# Install some other packages from source
wstool init src
wstool merge -t src src/complete_coverage/.rosinstall
wstool update -t src

# Get dependencies through rosdep (note that we are skipping python-serial because it is not available for ubuntu focal)
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --skip-keys="python-serial"

# We need to overlay the cartographer workspace in order to be able to use those packages
source ~/cartographer_ws/install_isolated/setup.bash 

catkin_make
source devel/setup.bash
```

Note that we are overlaying the Cartographer workspace `cartographer_ws`.

You can run `catkin_make --force-cmake` to verify that it was overlayed correcly. You should be able to see this in the output:
```
-- This workspace overlays: /home/username/cartographer_ws/install_isolated;/opt/ros/noetic
```

## Simulation
To simulate the USV, we use the [usv_simulator](https://github.com/jhlenes/usv_simulator.git) package.

Start the simulation by running
```
roslaunch otter_gazebo otter.launch
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
Once either of these are started while the ```guidance``` and `simulation` are running, the USV should start moving.

## Other documentation

These things are not necessary to get the simulation up and running.

### Sensors
This system will connect to several sensors. In order to know which sensor is at which USB port (```/dev/ttyUSB*```), we need to set up some udev rules. These rules are defined for [RPLIDAR](etc/rplidar.rules), and [Xsens IMU](etc/xsens.rules). Apply them by:
```
sudo cp etc/rplidar.rules /etc/udev/rules.d
sudo cp etc/xsens.rules /etc/udev/rules.d
```
Unplug and replug your devices, and you are finished.

If you would like to add other sensors as well, you can find the ``` ATTRS{idVendor}``` and ```ATTRS{idProduct}``` with the ```lsusb``` command. If the names are cryptic, just disconnect and reconnect your devices to see what is changing.

### Map inflating
The ```map_inflating``` package uses a [costmap](http://wiki.ros.org/costmap_2d) in order to inflate nearby obstacles.

### SLAM
The ```slam``` package performs simultaneous localization and mapping (SLAM) with [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/).

