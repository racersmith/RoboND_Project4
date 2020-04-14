# Udacity Robotics Nanodegree
## Project 4: SLAM
Simultaneous localization and mapping using Real-time Appearance-Based Mapping or RTAB. 

![RTAB SLAM](/media/RTAB_Mapping.png?raw=true "RTAB SLAM in progress")

#### Setup
``` shell
git clone https://github.com/racersmith/RoboND_Project4
cd RoboND_Project4/catkin_ws
catkin_make
source devel/setup.bash
```

This will clone, build and source the project.  Now, we need to launch all of the nodes for the localization demo.  This can be done with the main launch file.

1. Launch the world
``` shell
roslaunch my_robot world.launch
```

2. Launch RTAB
``` shell
roslaunch my_robot mapping.launch
```

3. Launch the teleop
``` shell
roslaunch my_robot teleop.launch
```
