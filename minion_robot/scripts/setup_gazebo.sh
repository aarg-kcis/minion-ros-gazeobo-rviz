#!/bin/bash


ROBOS=$1

# ROBOT_IDS="["
# HUMAN_INPUT="[1"

Xs=( -7 -5 -3 -1 1 3 5 7 9 11)
Ys=(  0  0  0  0 0 0  0  0  0  0)

if [ $# -ne 1 ]; then
        echo "usage: $0 <number of robots>"
        exit
fi



# roslaunch random_moving_target spawn_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &
# for i in $(seq 0 $(($ROBOS-1))); do
# 	id=$(($i+1))
# 	roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=5 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen &
# 	rosrun hkt_experiments uav_state_tf_closer $id &
# done


date
#roslaunch minion_robot gazebo.launch --screen &

#sleep 3

for i in $(seq 0 $(($ROBOS-1))); do
  id=$(($i+1))
  roslaunch minion_robot minion_with_ID.launch roboID:=$id z:=0.1 x:=${Xs[$i]}  y:=${Ys[$i]} --screen &
  sleep 5
  #roslaunch minion_robot minion_nodes.launch --screen &

done



#roslaunch minion_robot minion_battery_plugin.launch  --screen &
