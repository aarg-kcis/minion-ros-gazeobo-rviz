# minion-ros-gazeobo-rviz

# Battery Plugin: Clone into catkin workspace src 
$ git clone https://github.com/pooyanjamshidi/brass_gazebo_battery.git
$ catkin_make
$ . devel/setup.bash

# Actuator Plugins: Save the Gazebo_Plugins folder outside your catkin workspace
$ cd Gazebo_Plugins
$ mkdir build
$ cd build
$ cmake ../
$ make

# Open 3 terminals
# first terminal: 
$ cd catkin_ws/src/minion_robot/scripts
$./setup_gazebo.sh 

# second terminal run: 
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" 

# To Kill Process: 
$ cd catkin_ws/src/minion_robot/scripts
$./cleanup.sh 

For limits on the input velocity: Check diff_drive_controller.yaml Tutorials followed : http://wiki.ros.org/urdf/Tutorials,  http://gazebosim.org/tutorials
