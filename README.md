# minion-ros-gazeobo-rviz
Only a basic model. You can append meshes and .stl cad files to the model to make it look like the real robot: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch

# Open 3 terminals
# first terminal run: 
roslaunch minion_robot controller_gazebo.launch 

(above command runs a differential drive controller+(rviz and gazebo files))

# second terminal run: 
rostopic pub -r 10 /minion_robot_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" 

# third terminal: 
rostopic echo /minion_robot_controller/odom

For limits on the input velocity: Check diff_drive_controller.yamlTutorials followed : http://wiki.ros.org/urdf/Tutorials,  http://gazebosim.org/tutorials
