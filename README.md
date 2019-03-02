# otter_ros
This is a collection of ROS packages.

## Sensors
This system will connect to several sensors. In order to know which sensor is at which USB port (```/dev/ttyUSB*```), we need to set up some udev rules. These rules are defined for [RPLIDAR](etc/rplidar.rules), and [Xsens IMU](etc/xsens.rules). Apply them by:
```
sudo cp etc/rplidar.rules /etc/udev/rules.d
sudo cp etc/xsens.rules /etc/udev/rules.d
```
Unplug and replug your devices, and you are finished.

If you would like to add other sensors as well, you can find the ``` ATTRS{idVendor}``` and ```ATTRS{idProduct}``` with the ```lsusb``` command. If the names are cryptic, just unplug and replug your devices to see what is changing.

## Installation

This package was made with ROS Kinetic. If you need help installing this package, look through the Dockerfile. Or just use Docker.

## Docker

A Docker image has been created with a complete ROS installation on a Jessie image.
### Build the Docker image
```
docker build -t rosapp .
```

### Give the docker usergroup permission to use X Server
```
xhost +local:docker
```
This makes it possible to use GUI applications within the docker image. Use ```root``` instead of ```docker``` if you have not set up a docker usergroup.

### Run 
```
docker run -it --device="/dev/ttyUSB0" --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:rw --name ros --env="QT_X11_NO_MITSHM=1" rosapp
```
If you don't have a lidar connected use this instead:
```
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:rw --name ros --env="QT_X11_NO_MITSHM=1" rosapp
```

### Open running container in several terminals
In a new terminal, run
```
docker exec -it ros bash
```

### Setup environment
To source the ROS environment, run
```
source /setup.bash
```
You can now run the commands in the sections below.

## Simulation

A simulation environment has been made with Gazebo. The simulation is currently of a wheeled robot equipped with a laser scanner sensor / lidar. The world is flat and contains some obstacles.
The plan is to change this simulation so that it actually simulates the Otter USV on water instead of a wheeled robot. However, for simple testing of algorithms, the current simulation is good enough.

The simulation is split up in three packages:

- `otter_control` contains the configuration of the movement of the simulated robot. Here you can for instance change the speed at which it can move.
- `otter_description` contains the description of the robot, i.e. how it looks. Also contains laser scanner configurations.
- `otter_gazebo` contains the description of the world which is simulated.

### Start the simulation
```
roslaunch otter_gazebo otter.launch
```
If you want to launch with a GUI to see how the simulation looks in Gazebo
```
roslaunch otter_gazebo otter.launch gui:=true
```
The first time usually takes a little while to load.

## SLAM

## Navigation

## Coverage

