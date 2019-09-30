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
### Use:
* `$ roscore`
* `$ rosrun gazebo_ros gazebo basicSimulation.world --verbose`
* Use rviz and rostopic to see the sensor data stream out.
