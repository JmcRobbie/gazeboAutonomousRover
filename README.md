# gazeboAutonomousRover
A repo to manage the codebase for the testing of a rover in a simulated space. 
## Dependencies: 

Everything runs on Ubuntu 18.04 LTS

* Ros melodic 
* Gazebo http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
* Gazebo_ros_pkgs - install by command:
  `sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`
* `ros-melodic-hector-gazebo-plugins`
* Ensure you checkout this repo to src folder of your catkin workspace.
* Use `catkin build` anywhere within workspace to build your packages.
* Gazebo searches for models using env variable `GAZEBO_MODEL_PATH`. Therefore, you need to add the full path of the models folder to the variable.
`echo "export GAZEBO_MODEL_PATH=/path/to/models/:$GAZEBO_MODEL_PATH" >> ~/.bashrc`

### Optional tools
* the ros rqt robot control tool for steering the rover via a gui: `$ sudo apt-get install ros-melodic-rqt-robot-steering`

### Use:
* `$ roscore`
* `$ rosrun gazebo_ros gazebo path/to/basicSimulation.world --verbose`
* Or can run launch file `roslaunch autonomous basiSimulation.launch` - requires rqt-robot-steering to be installed
* Use rviz and rostopic to see the sensor data stream out.
* Skid Steer Drive Controller plugin is used for primitive driving of rover. 
  * Can be driven graphically using `$ rosrun rqt_robot_steering rqt_robot_steering` or by publishing to `/cmd_vel` topic: `$ rostopic pub  /cmd_vel geometry_msgs/Twist '[<linear_speed>, 0, 0]' '[0, 0, <angular_speed>]'`
  * Also provides odometry data on topic `\odom`
