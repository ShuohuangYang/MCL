# Multi-Robot Colloaborative Localization

## Cloning: 
Because of the submodules, you will need to clone with the following command: `git clone --recurse-submodules https://github.com/ShuohuangYang/MCL.git`

## Installing prerequisites:
From the root of the directory, run: `rosdep install --from-paths src --ignore-src -r -y`

## Compiling:
From the root of the directory, run: `catkin_make_isolated`

## Simulatuions:
Simulations are based on the turtlebot3 tutorials and minor changes are needed.
(We need to combine these into a launch file later!!)
### To launch the Gazebo environment:

```
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

### Image process node:
`ROS_NAMESPACE=tb3_0/rrbot/camera1 rosrun image_proc image_proc`
### Apriltag node:
`roslaunch apriltag_ros continuous_detection.launch`
