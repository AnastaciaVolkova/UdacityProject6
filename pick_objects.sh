#!/bin/sh
cd ../
xterm -e "source ./devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source ./devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "source ./devel/setup.bash; roslaunch pick_objects pick_objects_node 0" &
sleep5

