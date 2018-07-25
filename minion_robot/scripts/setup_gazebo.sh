#!/bin/bash


ROBOS=$2

# ROBOT_IDS="["
# HUMAN_INPUT="[1"

Xs=( -7 -3  1 4 7 7 8  10 0  2)
Ys=(  0  0  0 0 3 5 8  5  7 -5)

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
#roslaunch random_moving_target spawn_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &
for i in $(seq 0 $(($ROBOS-1))); do
  id=$(($i+1))
  roslaunch minion_robot minion_with_ID.launch roboID:=$id z:=0.1 x:=${Xs[$i]}  y:=${Ys[$i]} --screen & #spawn robot and low-level controller
  # roslaunch minion_robot g2g.launch roboID:=$id --screen &
  #roslaunch pure_pursuit pure_pursuit_controller.launch robot_name:=minion roboID:=$id --screen &                          #go to goal controller
done


# rostopic pub minion_$id/path_segment nav_msgs/Path "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: 'world'
# poses:
# - header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs: 0
#     frame_id: 'world'
#   pose:
#     position:
#       x: 10
#       y: 10
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 0.0" --screen &                                                                          #demo of pure_pursuit
