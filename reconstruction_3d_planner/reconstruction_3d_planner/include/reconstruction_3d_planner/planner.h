#ifndef RECONSTRUCTION_3D_PLANNER__PLANNER_H_
#define RECONSTRUCTION_3D_PLANNER__PLANNER_H_

// C++
#include <cmath>
#include <limits>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <octomap_msgs/EstimateGain.h>
#include <reconstruction_3d_planner/OptimizeCurrentPose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>


namespace planner
{
  /**
   * @class Planner
   * @brief A class that receives a PoseStamped and sends it to move_base
   * optimizing the pose of the camera along the trajectory of the robot
   */
  class Planner
  {
    public:
      Planner();
      ~Planner();

    private:
      void printTF(const tf::Transform echo_transform, int precision = 3);
      inline float only2decimals(float value){return roundf(value * 100) / 100;};
      bool optimizePan(float time, float& pan_optimal, tf::Transform& tf_optimal_global_sensor);
      bool optimizeCurrentPoseCb(reconstruction_3d_planner::OptimizeCurrentPose::Request &req, reconstruction_3d_planner::OptimizeCurrentPose::Response &res);
      void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
      void activeCallback();
      void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
      void doneCallback(const actionlib::SimpleClientGoalState& state,
                                 const move_base_msgs::MoveBaseResultConstPtr& result);


      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;
      ros::Subscriber sub_goal_;
      ros::ServiceClient srvc_plan_;
      ros::ServiceClient srvc_estimate_gain_;
      ros::ServiceServer srvs_optimize_current_pose_;
      ros::Publisher sensor_traj_pub_;
      ros::Publisher sensor_pos_pub_;
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
      geometry_msgs::PoseStamped current_goal_;
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;
      ros::Subscriber sub_joint_states_;
      std::string base_frame_;
      std::string global_frame_;
      std::string sensor_frame_;
      float costmap_resolution_;
      float base_max_trans_vel_;
      // Field of view parameters
      float h_res_;
      float v_res_;
      float angle_h_min_;
      float angle_h_max_;
      float angle_v_min_;
      float angle_v_max_;
      float range_min_;
      float range_max_;
      float fov_rays_;
      // Optimization parameters
      float unknown_gain_value_;
      bool optimize_pan_;
      float pan_vel_;
      float pan_min_;
      float pan_max_;
      float pan_step_;
      std::string pan_joint_name_;
      std::string pan_link_name_;
      float current_pan_;
      // Goal selection parameters
      float goal_selection_radius_;

      bool goal_running_;

//      bool sensor_to_left_;  // test

  };


}  // namespace planner

#endif
