# otter_ros
This is a collection of ROS packages.

## Simulation

A simulation environment has been made with Gazebo. The simulation is currently of a wheeled robot equipped with a laser scanner sensor / lidar. The world is flat and contains some obstacles.
The plan is to change this simulation so that the it actually simulates the Otter USV on water instead of a wheeled robot. However, for simple testing of algorithms, the current simulation is good enough.

The simulation is split up three packages:

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

## TODO
