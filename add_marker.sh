#!/bin/sh
cd ../
xterm -e "source ./devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source ./devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "srouce ./devel/setup.bash; rosrun add_markers add_markers_node -9.0 -5.0 -4.0 3.0" &
sleep 5
