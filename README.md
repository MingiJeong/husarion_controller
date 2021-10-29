# husarion_controller

* Author: Mingi Jeong (TA)
* Prof: Alberto Quattrini Li
* class: COSC 81/281 TA 2021 Fall
* License: MIT

## Dependencies
* numpy
* stage_ros OR husarion ROSBOT 2.0
* ROS Melodic and Ubuntu 18.04

## Purpose
This package enables ```rosbot_controller``` as well as ```occupancy grid mapper```.


## Installation
assuming you are using ```stage_ros``` or real ```ROSBOT_2```,
do the following steps into your ROS workspace.

```bash
git clone https://github.com/MingiJeong/husarion_controller.git
```

```bash
cd to your workspace
catkin_make

source your workspace/devel/setup.bash
```

## Usage

* Note that the performance is not optimized but only implementation for instruction of the students from teaching members.
* Make sure you run the simulator or relevant node (ekf, hardware.launch) on the rosbot first after ```ssh```. 

### 1. Configuration
* In ```constant.py```, If you are using a rosbot, change the ``laser scan`` topic name and frame accordingl, i.e., ```scan```, ```laser```, different from the simulator.
* In ```setup.yaml```, change the map data according to your preference. 


### 2. Main controller
* The node is originally running ``straight``, ``turn +- 30 deg``, ``turn abs motion`` for the testing.

```bash
roslaunch husarion_controller main_controller.launch
```

### 3. Mapper
* The node is running occupancy grid mapper based on Bresenham's Line Algorithm.

```bash
roslaunch husarion_controller mapper.launch
```

![screenshot](/images/screenshot2.png)
