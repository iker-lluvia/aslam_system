/* Copyright (C) 2021,  Tekniker - All Rights Reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 *
 * Written by Iker Lluvia <iker.lluvia@tekniker.es>
 */


// ROS
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>


std::string model_name_;
ros::Publisher pub_set_model_state_;

// Topic callback
void gazeboModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& model_states)
{
  bool found = false;
  int i = 0;
  while (!found)
  {
    if (!strcmp(model_states->name[i].c_str(), model_name_.c_str()))
    {
      gazebo_msgs::ModelState model_state;
      model_state.model_name = model_states->name[i];
      model_state.pose = model_states->pose[i];
      model_state.twist = model_states->twist[i];
      pub_set_model_state_.publish(model_state);
      found = true;
    }
    ++i;
  }
}


int main(int argc, char** argv)
{
  // ROS node initialization
  ros::init(argc, argv, "replicate_model_states");
  
  // Node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Parameter getters
  nh_private.param<std::string>("model_name", model_name_, "turtlebot3_waffle");
  
  // Publishers
  pub_set_model_state_ = nh_private.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  // Subscribers
  ros::Subscriber sub = nh.subscribe("/bag/gazebo/model_states", 1000, gazeboModelStatesCb);
  
  ros::spin();
  
  return 0;
}
