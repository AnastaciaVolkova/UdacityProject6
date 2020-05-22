# Udacity Project6 "Home Service Robot"

Course Project from Robotics Software Engineer - Path planning and navigation.

localization system for a robot should be installed:
sudo apt-get install ros-melodic-navigation

## Descirption
### Scripts
- test_slam.sh
- test_navigation.sh

#### <a name="test_slam"></a>test_slam.sh
Script does the following:

- Launches _world.launch_ (from package _my_robot_). <p>With the help of this launch-file an environment starts, which I create with gazebo. This is a model of my appartment (_wohnung.world_). Moreover my robot is loaded. I create the robot in  Unified Robotic Description Format (URDF) (_my_robot.xacro_). Nodes, which publish state of robot and joint states are also launched. Finally rviz, a 3D visualization tool for ROS, starts.</p>
- Launches _slam_gmapping_pr2.launch_ (from package _slam_gmapping_). <p> With the help of launch configuration file node _slam_gmapping_ is run. The node uses OpenSlam and provides laser-based SLAM. With usage of _slam_gmapping_, I  create a 2-D occupancy grid map. </p>
- Launches _teleop_twist_keyboard.py_ (from package  _teleop_twist_keyboard_). <p> With this node I drive a robot in my environment, in order to gather information about obstacles (mostly walls) in my appartment. </p>

**This script is used to create an environment map, which I will use further in navigation.**  I create the bitmap representation of map with the help of _map_server_.
The information, which was gathered by _slam_gmapping_, is provided by topic _map (nav_msgs/OccupancyGrid)_ to _map_server_. Node map_server creates a bitmap represantation and yaml file, which describes parameters of map.
Map (wohnung.pgm and wohnung.yaml) is saved in ./my_robot/map/

#### test_navigation
Script does the following:

- Launches _world.launch_ (from package _my_robot_). <p>See description from test_slam.sh</p>
- Launches _amcl.launch_. <p>The file starts _map_server_ node, _amcl_ from package _navigation_  node, _move_base_ from _navigation_ package node. Node _amcl_ realizes probabilistic localization of a robot in 2D. To track the pose of a robot against a known map, it uses Adaptive (or KLD-sampling) Monte Carlo algorithm. _move_base_ is a node which controls the robot movements, so that it can reaches the goal (so the goal should be provided). It uses components global_planner, global_costmap, local_planner, local_costmap, recovery_behaviours for creating a paths.</p>
- Runs _map_server_ node, which provides a map to _move_base_.

**The script is used to manually set goal and navigate to it.**

