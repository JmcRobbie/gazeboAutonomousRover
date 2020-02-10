# gazebo autonomous simulation for Rover
A repo to manage the codebase for the testing of a rover in a simulated space. 



![alt text][logo]

[logo]: https://github.com/JmcRobbie/gazeboAutonomousRover/doc/imgs/demo_img.png "Simulated rover in mars environment"

Clone this repo to src folder of your catkin workspace. 
## Environment and Dependencies: 

Everything runs on Ubuntu 18.04 LTS

* Install ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
* Install catkin: `$ sudo apt-get install ros-melodic-catkin`
* Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
* If the ROS installation didn't install gazebo then follow: http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
* Install Gazebo_ros_pkgs:
  * `$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`
  * `$ sudo apt-get install ros-melodic-hector-gazebo-plugins`
  * `$ sudo apt-get install ros-melodic-rqt-robot-steering` - pkg for steering rover via a gui
* Depends on messages defined in common repo https://github.com/novarover/common, please clone into same workspace.
  
 ## Git Usage:
 * Clone this repository into a subdirectory of the catkin workspace src folder.
 * To develop a feature: branch from `develop` and make a pull request into `develop` when finished with changes.

 ## Compiling:
* Use `$ catkin build` anywhere within workspace to build your packages. If you've previously used `catkin_make` then you may need to delete build and devel folders from your workspace.
* For ROS to see your packages you must use command `$ source ~/catkin_ws/devel/setup.bash` where `catkin_ws` is the path of your workspace folder. This can be done automatically with each new terminal by adding to .bashrc: `$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
* Gazebo searches for models using env variable `GAZEBO_MODEL_PATH`. Therefore, you need to add the full path of the models folder to the variable.
`$ echo "export GAZEBO_MODEL_PATH=/path/to/models/:$GAZEBO_MODEL_PATH" >> ~/.bashrc`

## Use:
* Run launch file `$ roslaunch autonomous_sim basicSimulation.launch` 
* Can also run .world file directly after running `$ roscore`: `$ rosrun gazebo_ros gazebo path/to/basicSimulation.world --verbose`.
* Use rviz and rostopic to see the sensor data stream out.
* Skid Steer Drive Controller plugin is used for primitive driving of rover. 
  * Can be driven graphically from launch file or using `$ rosrun rqt_robot_steering rqt_robot_steering`. 
  * Can drive by publishing to `/cmd_vel` topic: `$ rostopic pub  /cmd_vel geometry_msgs/Twist '[<LINEAR_SPEED>, 0, 0]' '[0, 0, <ANGULAR_SPEED>]'`
  * Can drive by publishing to `/gazebo/driver/drive_cmd` topic: `$ rostopic pub /gazebo/driver/drive_cmd /gazebo/driver/drive_cmd autonomous_sim/DriveCmd '{rpm: <RPM>, steer_pct: <STEER_PCT>}`
  * Also provides odometry data on topic `/odom`
  * Encoder data of wheels is published to `/gazebo/encoder_data`
  * Laser scan data is published to `/scan`

* Robot models provided are basic_rover and laser_rover. To launch the basic model in the closed_maze.world provided, use command:
  * `$ roslaunch autonomous_sim closed_maze.launch`
* The choice of robot model may be passed into the launch file as an argument of model, like so:
  * Launch 3D LIDAR rover in maze world: `$ roslaunch autonomous_sim closed_maze.launch model:=laser_rover`
  * Launch 2D LIDAR rover in empty world:`$ roslaunch autonomous_sim empty_world.launch model:=2d_lidar_rover`
* The competition world contains all 7 legs of the competition with the corresponding ar tags. Use `$ roslaunch autonomous_sim competition.launch` to launch the world with basic rover (or override with `model:=`).
* With the autonomous package from the https://github.com/novarover/autonomous repo (checkout to the correct branch), `$ roslaunch autonomous simulation_bug2.launch` to run simulation of bug 2 motion planning algorithm. 

