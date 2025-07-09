#!/bin/bash

gnome-terminal -x bash -c "roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.31.6:20100"; exec bash;"
sleep 3s

gnome-terminal -x bash -c  "cd $HOME/catkin_match/src/sensor_pkg; python3 main.py; exec bash"
sleep 3s
gnome-terminal --tab  -e 'bash -c "source $HOME/catkin_match/devel/setup.bash; roslaunch faster_lio rflysim_sim.launch rviz:=false; exec bash;"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/catkin_match/devel/setup.bash; python3 $HOME/catkin_match/src/object_det/scripts/det.py; exec bash"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/catkin_match/devel/setup.bash; roslaunch ego_planner RflySimDepth.launch; exec bash"'
sleep 5s

gnome-terminal --tab  -e 'bash -c "source $HOME/catkin_match/devel/setup.bash; roslaunch mission_pkg match_nodetect.launch; exec bash"'
sleep 5s

gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /move_base_simple/goal; exec bash;"'
# sleep 3s
# gnome-terminal --window  -e 'bash -c "roslaunch mission_pkg match.launch; exec bash;"'
#sleep 3s
