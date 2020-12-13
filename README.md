# Multi-Robot Colloaborative Localization

## Cloning: 
Because of the submodules, you will need to clone with the following command: `git clone --recurse-submodules https://github.com/ShuohuangYang/MCL.git`

## Installing prerequisites:
From the root of the directory, run: `rosdep install --from-paths src --ignore-src -r -y`

## Modified files 
To satisfy project target, several files under submodule are modified. To change the files as required, checkout to branch `demo` and download those changed files under `src_changed`, then substitue original files with corresponding ones.

## Compiling:
From the root of the directory, run: `catkin_make_isolated`

## Simulations:
Simulations are based on the turtlebot3 tutorials and minor changes are needed.
(We need to combine these into a launch file later!!)
### To launch the Gazebo environment:

```
export TURTLEBOT3_MODEL=burger
source devel_isolated/setup.bash
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

### And to launch the visualizer:
`rosrun rviz rviz`
Then from rviz, open the rviz config file in the mcl/config folder.

### Image process node:
`ROS_NAMESPACE=tb3_1/rrbot/camera1 rosrun image_proc image_proc`
`ROS_NAMESPACE=tb3_2/rrbot/camera2 rosrun image_proc image_proc`
### Apriltag node:
`roslaunch apriltag_ros continuous_detection.launch`
