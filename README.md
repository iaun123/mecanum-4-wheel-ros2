# Autonomous Mobile Robot Mecanum four wheels with ROS2 (MtET Thesis)
Ubuntu 20.04.6 LTS (Focal Fossa) ros2 foxy\
you can read about ros2 foxy from this link [https://docs.ros.org/en/foxy/Installation.html] and you can read about nav2 form this link [https://docs.nav2.org/].
## set model on gazebo
download robot model and then add models in folder .gazebo
## install
install package for use function in ros2 if you use another version use changed foxy to your version
### iron_x pkg
$ sudo apt install ros-foxy-joint-state-publisher-gui\
$ sudo apt install ros-foxy-xacro\
$ sudo apt install ros-foxy-robot-localization\
$ sudo apt install ros-foxy-gazebo-ros-pkgs\
$ sudo apt install ros-foxy-slam-toolbox
### nav2
$ sudo apt install ros-foxy-navigation2\
$ sudo apt install ros-foxy-nav2-bringup
### joy
$ sudo apt install ros-foxy-joy
### point clond
$ sudo apt install ros-foxy-rtabmap*\
$ sudo apt install ros-foxy-pcl-ros\
$ sudo apt install pcl-tools
