# Udacity Project6 "Home Service Robot"

Course Project from Robotics Software Engineer - Path planning and navigation.

localization system for a robot should be installed:
sudo apt-get install ros-melodic-navigation

## Descirption
### Scripts
- [test_slam.sh](#test_slam)
- [test_navigation.sh](#test_navigation)

#### <a name="test_slam"></a>test_slam.sh
Script does the following:

- <details><summary>launches _world.launch_ (from package _my_robot_)</summary>
<p>
Launches environment, which I created with gazebo. This is a model of my appartment (_wohnung.world_). Loads my robot, which I created in  Unified Robotic Description Format (URDF) (_my_robot.xacro_). Launches nodes, which publish state of robot and joint states. Launches rviz, a 3D visualization tool for ROS.
</p>
</details>
- <details> <summary>launches _slam_gmapping_pr2.launch_ (from package _slam_gmapping_)</summary>
<p>
Launches slam_gmapping_pr2.launch from slam_gmapping. 
It means that node _slam_gmapping_ is run. The node uses OpenSlam and provides laser-based SLAM. With usage of _slam_gmapping_, I  create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. 
</p>
</details>
- <details><summary>launches _teleop_twist_keyboard.py_ (from package  _teleop_twist_keyboard_)</summary>
<p>
With this node I drive a robot in my environment, in order to gather information about obstacles (mostly walls) in my appartment.
</p>
</details>
<text style="color:FireBrick">This script is used to create an environment map, which I will use further in navigation. </text> I created the map with help of _map_server_.
The information, which was gathered by _slam_gmapping_, is provided by topic _map (nav_msgs/OccupancyGrid)_ to _map_server_. Node map_server creates a bitmap represantation and yaml file, which describes parameters of map.
Map (wohnung.pgm and wohnung.yaml) is saved in ./my_robot/map/

#### <a name="test_navigation"></a>test_navigation.sh
Script does the following:

- launches _my_robot world.launch_ (from package _my_robot_)
- <details><summary>launches _my_robot amcl.launch_</summary>
The files launches the following nodes:
map_server
<p style="background-color:Lavender">_amcl_ from package _navigation_ is a node, which realizes probabilistic localization of a robot in 2D. To track the pose of a robot against a known map, it uses Adaptive (or KLD-sampling) Monte Carlo algorithm.</cite>
<p style="background-color:MintCream">
_move_base_ from _navigation_ package is a node which controls the robot movements, so that it can reaches the goal (so the goal should be provided). It uses components global_planner, global_costmap, local_planner, local_costmap, recovery_behaviours for creating a pathes.
</p>
- map_server - a node, which provides a map to _move_base_
</details>
<text style="color:FireBrick"> The script is used to manually set goal and navigate to it. </text>

