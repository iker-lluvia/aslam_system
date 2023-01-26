# ASLAM system

We present an active simultaneous localisation and mapping system that optimises the pose of the sensor for the 3D reconstruction of an environment, while a 2D Rapidly-Exploring Random Tree algorithm controls the motion of the mobile platform for the ground exploration strategy. You can find a video demo in simulation here: https://www.youtube.com/watch?v=NAPj3_A2e74.

## Installation

The code has been only tested in Ubuntu 18.04 with ROS Melodic.\
To use it, the steps are those common to any ROS package, which are:

1. Move and install outside the ROS workspace the uncertainty octomap package: aslam_system/octomap

2. Compile the workspace where this repository is located.

Note: some dependencies may have to be installed manually.

## Launch
1. Run main launch file, which includes yocs_cmd_vel_mux, slam_gmapping, move_base_turtlebot3_omni, octomap_uncertainty_server_node, rviz:\
`roslaunch aslam_bringup aslam_system.launch`

2. Run 2D RRT exploration:\
`roslaunch aslam_bringup rrt_simple.launch`

3. Run optimal camera pose planner:\
`roslaunch aslam_bringup planner.launch`

Here, we should have all the required nodes and services ready. To start exploration:

6. Start calling the service **/optimize_current_pose** (offered by the planner) to estimate the camera's next best pose for exploration. To call it in a loop, use the script:\
`roscd aslam_bringup/`\
`./optimize_service_loop.sh`

7. Start the 2D exploration system. You can follow the instructions from [here](https://wiki.ros.org/rrt_exploration/Tutorials/singleRobot#Start_Exploration).\
It requieres 5 points to be published in the /clicked_point topic. It can be done using rviz, and must follow this order:

> 1. top-left
> 2. bottom-left
> 3. bottom-right
> 4. top-right
> 5. initial point

At this point, the robot should start navigating towards frontiers.

If the robot gets stucked, **teleop_key_turtlebot3.launch** allows to move it manually:\
`roslaunch aslam_bringup teleop_key_turtlebot3.launch`

To know the value representing the number of voxels added to the 3D map, the service **/octomap_server/get_map_voxels** can be called:\
`rosservice call /octomap_server/get_map_voxels "{}"`

To know the value representing the information offered by the 3D map, the service **/octomap_server/get_map_info** can be called:\
`rosservice call /octomap_server/get_map_info "{}"`

## Citation

If you use this code, please cite the related paper:

```latex
@article{lluvia2023camera,
  author={Lluvia, Iker and Lazkano, Elena and Ansuategi, Ander},
  journal={IEEE Access},
  publisher={IEEE},
  title={Camera pose optimisation for 3D mapping},
  year={2023},
  volume={},
  number={},
  pages={1-1},
  doi={10.1109/ACCESS.2023.3239657}
}
```

## Authors

Iker Lluvia. Autonomous and Intelligent Systems, [Tekniker](https://www.tekniker.es/es).
