# gazeboAutonomousRover
A repo to manage the codebase for the testing of a rover in a simulated space. 
## Dependencies: 

Everything runs on Ubuntu 18.04 LTS

*You will need to install*: 
* Ros melodic 
* Gazebo http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
* Gazebo_ros_pkgs - install by command:
  `sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`
* `ros-melodic-hector-gazebo-plugins`
* Gazebo searches for models using env variable `GAZEBO_MODEL_PATH`. Therefore, you need to add the parent directory of `basic_rover` to the path.
`echo "export GAZEBO_MODEL_PATH=/path/to/models/:$GAZEBO_MODEL_PATH" >> ~/.bashrc`

### Use:
* `$ roscore`
* `$ rosrun gazebo_ros gazebo basicSimulation.world --verbose`
* Use rviz and rostopic to see the sensor data stream out.
* Skid Steer Drive Controller plugin is used for primitive driving of rover. 
  * Can be driven graphically using `$ rosrun rqt_robot_steering rqt_robot_steering` or by publishing to `/cmd_vel` topic: `$ rostopic pub  /cmd_vel geometry_msgs/Twist '[<linear_speed>, 0, 0]' '[0, 0, <angular_speed>]'`
  * Also provides odometry data on topic `\odom`
