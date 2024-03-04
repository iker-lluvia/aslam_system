# ASLAM System

Stuff for a novel ASLAM system.

## Requirements

1. [Ubuntu 18.04](https://ubuntu.com/tutorials/install-ubuntu-desktop-1804#1-overview)
2. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Installation

0. Install ROS dependencies:<br />
`sudo apt install -y ros-melodic-navigation ros-melodic-gmapping`<br />

1. Clone this repository<br />
`mkdir -p ~/ROS && cd ~/ROS`<br />
`git clone --recurse-submodules -j8 https://gitlab.tekniker.es/sai/personal/illuvia/freiburg/aslam-system.git`<br />
Note: you can use a [git cache](https://stackoverflow.com/a/5343146) not to enter credentials for every single submodule.<br />
3. Move non-ROS packages (octomap) to another folder:<br />
`mv aslam-system/octomap ~/Software/`<br />
2.[Create a ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and move ROS packages there:<br />
`mv aslam-system/*/ ~/ROS/aslam_ws/src/`<br />
3. Compile [octomap](https://gitlab.tekniker.es/sai/personal/illuvia/freiburg/octomap) (octomap and octovis):<br />
`cd ~/Software/octomap`<br />
`mkdir octomap/build && cd octomap/build`<br />
`cmake ..`<br />
`make -j8`<br />
`sudo make install`<br />
`cd ../..`<br />
`mkdir octovis/build && cd octovis/build`<br />
`cmake ..`<br />
Note: you may need to install QGLViewer `sudo apt install libqglviewer-dev-qt5`<br />
`make -j8`<br />
4. Compile ROS workspace:<br />
`roscd && cd .. && catkin_make`<br />
Note: it may give "fatal error: octomap_msgs/Octomap.h: No such file or directory", just recompile.<br />

## Running

Set Turtlebot model as an environment variable:<br />
`export TURTLEBOT3_MODEL=waffle`<br />
or add it to the ~/.bashrc:<br />
`echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc && source ~/.bashrc`<br />

### Simulation

1. Run the main simulation launch file:<br />
`roslaunch aslam_bringup aslam_system_sim.launch`<br />

### WidowX MX-28 turret + RealSense D435 camera

1. [Install RealSense dependencies](https://github.com/IntelRealSense/realsense-ros/tree/2.3.2#installation-instructions) (preferably, from [Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))<br />

2. Install [WidowX MX-28 turret's controllers](https://github.com/RobotnikAutomation/widowx_turret) and setup udev rules.<br />

3. TODO
