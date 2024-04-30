## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [map_manager](https://github.com/Zhefan-Xu/map_manager) which provides the occupancy voxel map implementation and [octomap](http://wiki.ros.org/octomap) for octree-based map. It also depends on [global_planner](https://github.com/Zhefan-Xu/global_planner) for global waypoint generation.

```
# install dependency
sudo apt install ros-[melodic/noetic]-octomap* # octomap

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/trajectory_planner.git

cd ~/catkin_ws
catkin_make
```

## II. Run Planner DEMO

```
roslaunch trajectory_planner mpc_interactive.launch
```
