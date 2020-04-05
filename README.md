# Udacity Robotics Nanodegree
## Project 3: Localization
Localize a robotic platorm using laser scans and wheel odometry to a known map using Adaptive Monte Carlo Localization, AMCL.

#### Setup
``` shell
git clone https://github.com/racersmith/RoboND_Project3
cd RoboND_Project3/catkin_ws
catkin_make
source devel/setup.bash
```

This will clone, build and source the project.  Now, we need to launch all of the nodes for the localization demo.  This can be done with the main launch file.

``` shell
roslaunch main main.launch
```

This will launch the Gazebo environment with our robot placed in the world and Rviz.  The Rviz is setup to display the robot, map, AMCL point cloud and laser range scan.

![Initial Estimate](/media/AMCL_t0.png?raw=true "Initial Estimate")

Now, to improve the localization estimate we need to get some additional data.  In Rviz, use the 2D Nav Goal to place a navigation target near the robot.  The robot will start to move towards the goal and as new sensor information is obtained for the new pose the estimate will start converging.

Here is an example convergence:
![Initial Estimate](/media/AMCL_t1.png?raw=true "Initial Estimate")
![Initial Estimate](/media/AMCL_t2.png?raw=true "Initial Estimate")
![Initial Estimate](/media/AMCL_t3.png?raw=true "Initial Estimate")
![Initial Estimate](/media/AMCL_t4.png?raw=true "Initial Estimate")
