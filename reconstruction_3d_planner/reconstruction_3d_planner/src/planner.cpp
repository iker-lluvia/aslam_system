
#include "reconstruction_3d_planner/planner.h"


namespace planner
{
  Planner::Planner() :
      nh_(),
      private_nh_("~"),
      tf_listener_(tf_buffer_),
      move_base_ac_("move_base", true),
      goal_running_(false),
      current_pan_(0.0),
//      sensor_to_left_(true),  // test
      base_frame_("base_link"),
      global_frame_("odom"),
      sensor_frame_("top_camera_rgb_optical_frame"),
      costmap_resolution_(0.1),
      base_max_trans_vel_(0.5),
      h_res_(48),
      v_res_(36),
      angle_h_min_(-0.5148721),  // -29.5°
      angle_h_max_(0.5148721),  // +29.5°
      angle_v_min_(-0.401426),  // -23°
      angle_v_max_(0.401426),  // +23°
      range_min_(0.5),
      range_max_(4.0),
      unknown_gain_value_(1.0),
      optimize_pan_(true)
  {
    // Read the parameters from the parameter server

    private_nh_.param<std::string>("base_frame", base_frame_, base_frame_);
    private_nh_.param<std::string>("global_frame", global_frame_, global_frame_);
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, sensor_frame_);
    // /home/tekniker/ROS/aslam_ws/src/aslam-system/aslam_bringup/launch/turtlebot3_simulation/urdf/turtlebot3_waffle.gazebo.xacro
    // https://www.reconstructme.net/qa_faqs/intel-realsense-r200-review/#:~:text=The%20R200%20is%20able%20to,an%20excellent%20sensor%20for%20ReconstructMe.
    // 59° H, 46° V, 70° D (Horizontal, Vertical, Diagonal), Between 0.5m and 4m
    private_nh_.param<float>("fov/resolution/horizontal", h_res_, h_res_);
    private_nh_.param<float>("fov/resolution/vertical", v_res_, v_res_);
    private_nh_.param<float>("fov/angle/horizontal/min", angle_h_min_, angle_h_min_);
    private_nh_.param<float>("fov/angle/horizontal/max", angle_h_max_, angle_h_max_);
    private_nh_.param<float>("fov/angle/vertical/min", angle_v_min_, angle_v_min_);
    private_nh_.param<float>("fov/angle/vertical/max", angle_v_max_, angle_v_max_);
    private_nh_.param<float>("fov/range/min", range_min_, range_min_);
    private_nh_.param<float>("fov/range/max", range_max_, range_max_);
    fov_rays_ = h_res_ * v_res_;
    ROS_INFO_STREAM("fov_rays_: " << fov_rays_);
    private_nh_.param<float>("unknown_gain_value", unknown_gain_value_, unknown_gain_value_);

    // TODO: read the dof to optimize from an array
    private_nh_.param("pan/optimize", optimize_pan_, optimize_pan_);
    if (optimize_pan_)
    {
      private_nh_.param<float>("pan/vel", pan_vel_, 2.0);  // 120º/s aprox
      private_nh_.param<float>("pan/min", pan_min_, 0.0);  // No limit
      private_nh_.param<float>("pan/max", pan_max_, 0.0);  // No limit
      private_nh_.param<float>("pan/step", pan_step_, 0.52); // 30°
      private_nh_.param<std::string>("pan/joint_name", pan_joint_name_, "telescopic_arm_continuous_joint");
      private_nh_.param<std::string>("pan/link_name", pan_link_name_, "telescopic_arm_extension_base_link");
    }

    private_nh_.param<float>("goal_selection_radius", goal_selection_radius_, 1.0);

    sub_goal_ = nh_.subscribe("exploration_goal", 1, &Planner::goalCallback, this);  // Receives an exploration goal and optimizes the trajectory to it
    sub_joint_states_ = nh_.subscribe("joint_states", 1, &Planner::jointStateCallback, this);

    srvc_plan_ = nh_.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
    //~ ROS_INFO_STREAM("Waiting for " << srvc_plan_.getService() << " service server to start...");
    //~ srvc_plan_.waitForExistence();
    // TODO: check move_base parameters availability
    nh_.param<float>("move_base/global_costmap/resolution", costmap_resolution_, costmap_resolution_);
    nh_.param<float>("move_base/DWAPlannerROS/max_trans_vel", base_max_trans_vel_, base_max_trans_vel_);
    
    srvc_estimate_gain_ = nh_.serviceClient<octomap_msgs::EstimateGain>("octomap_server/estimate_gain");
    ROS_INFO_STREAM("Waiting for " << srvc_estimate_gain_.getService() << " service server to start...");
    srvc_estimate_gain_.waitForExistence();

    sensor_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/sensor_trajectory_controller/command", 1);
    sensor_pos_pub_ = nh_.advertise<std_msgs::Float64>("/telescopic_arm_continuous_joint_position_controller/command", 1);

    //~ ROS_INFO_STREAM("Waiting for /move_base/goal action server to start...");
    //~ move_base_ac_.waitForServer();
    
    srvs_optimize_current_pose_ = nh_.advertiseService("optimize_current_pose", &Planner::optimizeCurrentPoseCb, this);
    
    ROS_INFO_STREAM("Planner initialized.");
  }


  Planner::~Planner(){}
  
  
  void Planner::printTF(const tf::Transform echo_transform, int precision)
  {
    std::cout.precision(precision);
    std::cout.setf(std::ios::fixed,std::ios::floatfield);
    double yaw, pitch, roll;
    echo_transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = echo_transform.getRotation();
    tf::Vector3 v = echo_transform.getOrigin();
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
             << q.getZ() << ", " << q.getW() << "]" << std::endl
             << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
             << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
  }
  
  
//  bool Planner::optimizePan(float time, float& pan_optimal, tf::Transform& tf_optimal_global_sensor)
//  {
//    bool calculated = false;
//    // Get the pose of the robot
//    geometry_msgs::TransformStamped tfs_global_joint;
//    geometry_msgs::TransformStamped tfs_joint_sensor;
//    try
//    {
//      tfs_global_joint = tf_buffer_.lookupTransform(global_frame_, pan_link_name_, ros::Time(0));
//      tfs_joint_sensor = tf_buffer_.lookupTransform(pan_link_name_, sensor_frame_, ros::Time(0));
//    }
//    catch (tf2::TransformException &ex)
//    {
//      ROS_ERROR("%s", ex.what());
//      //~ res.success = false;
//      //~ res.message = ex.what();
//      return false;
//    }
//    tf::Transform tf_global_joint;
//    tf_global_joint.setOrigin(tf::Vector3(tfs_global_joint.transform.translation.x, tfs_global_joint.transform.translation.y, tfs_global_joint.transform.translation.z));
//    tf_global_joint.setRotation(tf::Quaternion(tfs_global_joint.transform.rotation.x, tfs_global_joint.transform.rotation.y, tfs_global_joint.transform.rotation.z, tfs_global_joint.transform.rotation.w));
//
//    tf::Transform tf_joint_sensor;
//    tf_joint_sensor.setOrigin(tf::Vector3(tfs_joint_sensor.transform.translation.x, tfs_joint_sensor.transform.translation.y, tfs_joint_sensor.transform.translation.z));
//    tf_joint_sensor.setRotation(tf::Quaternion(tfs_joint_sensor.transform.rotation.x, tfs_joint_sensor.transform.rotation.y, tfs_joint_sensor.transform.rotation.z, tfs_joint_sensor.transform.rotation.w));
//
//    // Calculate sensor boundaries based on the time elapsed
//    float pan_min_local;
//    float pan_max_local;
//    if (time < 0.01)  // if no time limit
//    {
//      pan_min_local = -3.14;
//      pan_max_local = 3.14;
//
//    }
//    else
//    {
//      pan_min_local = only2decimals(current_pan_ - pan_vel_ * time);
//      pan_max_local = only2decimals(current_pan_ + pan_vel_ * time);
//      if (abs(pan_min_) > 0.01 || abs(pan_max_) > 0.01)  // if not revolute joint
//      {
//        if (pan_min_local < pan_min_)
//        {
//          pan_min_local = pan_min_;
//        }
//        if (pan_max_local > pan_max_)
//        {
//          pan_max_local = pan_max_;
//        }
//      }
//    }
//
//    // TODO: Estimate the optimal pan position for the 3d reconstruction
//    //       Use pan_min_local/pan_max_local to calculate all the possible poses and estimate their gains
//    float max_gain = 0.0;
//    int possible_pans = only2decimals((pan_max_local - pan_min_local) / pan_step_);
//    ROS_INFO_STREAM("\n pan_min_: " << pan_min_ <<
//                    "\n pan_max_: " << pan_max_ <<
//                    "\n current_pan_: " << current_pan_ <<
//                    "\n time: " << time <<
//                    "\n pan_min_local: " << pan_min_local <<
//                    "\n pan_max_local: " << pan_max_local <<
//                    "\n pan_step: " << pan_step_ <<
//                    "\n possible_pans: " << possible_pans);
//    for (int i = 0; i < possible_pans; ++i)
//    {
//      calculated = true;
//      float candidate_pan = only2decimals(pan_min_local + pan_step_ * i);
//      ROS_INFO_STREAM("candidate_pan(" << candidate_pan <<
//                      ") = pan_min_local(" << pan_min_local <<
//                      ") + pan_step_(" << pan_step_ <<
//                      ") * i(" << i << ")");
//
//      tf::Quaternion quat_aux;
//
//      tf::Transform tf_current_pan;
//      tf_current_pan.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//      tf::Quaternion q1;
//      q1.setRPY(-current_pan_, 0.0, 0.0);
//      tf_current_pan.setRotation(q1);
//
//      tf::Transform tf_candidate_pan;
//      tf_candidate_pan.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//      tf::Quaternion q3;
//      q3.setRPY(candidate_pan, 0.0, 0.0);
//      tf_candidate_pan.setRotation(q3);
//
//      tf::Transform tf_candidate_global_sensor;
//      tf_candidate_global_sensor = tf_global_joint * tf_current_pan * tf_candidate_pan * tf_joint_sensor;
//
//      octomap_msgs::EstimateGain srv_estimate_gain;
//      tf::poseTFToMsg(tf_candidate_global_sensor, srv_estimate_gain.request.pose.pose);
//      srv_estimate_gain.request.pose.header.stamp = ros::Time::now();
//      srv_estimate_gain.request.pose.header.frame_id = global_frame_;
//      srv_estimate_gain.request.h_res = h_res_;
//      srv_estimate_gain.request.v_res = v_res_;
//      srv_estimate_gain.request.angle_h_min = angle_h_min_;
//      srv_estimate_gain.request.angle_h_max = angle_h_max_;
//      srv_estimate_gain.request.angle_v_min = angle_v_min_;
//      srv_estimate_gain.request.angle_v_max = angle_v_max_;
//      srv_estimate_gain.request.range_min = range_min_;
//      srv_estimate_gain.request.range_max = range_max_;
//      srv_estimate_gain.request.unknown_gain_value = unknown_gain_value_;
//      if (srvc_estimate_gain_.call(srv_estimate_gain))
//      {
//        ROS_INFO_STREAM("srv_estimate_gain.response.gain: " << srv_estimate_gain.response.gain << "\tmax_gain: " << max_gain);
//        if (srv_estimate_gain.response.gain > max_gain)
//        {
//          tf_optimal_global_sensor = tf_candidate_global_sensor;
//          pan_optimal = candidate_pan;
//          ROS_INFO_STREAM("pan_optimal: " << pan_optimal);
//          max_gain = srv_estimate_gain.response.gain;
//          // TODO: if all are unknown do not find more
//          //~ if (max_gain == unknown_gain_value_ * fov_rays_)
//          //~ {
//            //~ ROS_INFO_STREAM("break");
//            //~ break;
//          //~ }
//        }
//      }
//      else
//      {
//        ROS_WARN_STREAM("Call to service " << srvc_estimate_gain_.getService() << " failed");
//      }
//    }
//    ROS_INFO_STREAM("END for (int i = 0; i < possible_pans; ++i)");
//    ROS_INFO_STREAM("pan_optimal: " << pan_optimal);
//    return calculated;
//  }
  
  
  // /optimize_current_pose callback
  bool Planner::optimizeCurrentPoseCb(reconstruction_3d_planner::OptimizeCurrentPose::Request &req, reconstruction_3d_planner::OptimizeCurrentPose::Response &res)
  {
    tf::Transform tf_optimal_global_sensor;
    float pan_optimal = current_pan_;
    bool calculated = false;
    if (optimize_pan_)
    {
      //~ calculated = optimizePan(req.time, pan_optimal, tf_optimal_global_sensor);
      
      {
        // Get the pose of the robot
        geometry_msgs::TransformStamped tfs_global_joint;
        geometry_msgs::TransformStamped tfs_joint_sensor;
        try
        {
          tfs_global_joint = tf_buffer_.lookupTransform(global_frame_, pan_link_name_, ros::Time(0));
          tfs_joint_sensor = tf_buffer_.lookupTransform(pan_link_name_, sensor_frame_, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          res.success = false;
          res.message = ex.what();
          return false;
        }
        tf::Transform tf_global_joint;
        tf_global_joint.setOrigin(tf::Vector3(tfs_global_joint.transform.translation.x, tfs_global_joint.transform.translation.y, tfs_global_joint.transform.translation.z));
        tf_global_joint.setRotation(tf::Quaternion(tfs_global_joint.transform.rotation.x, tfs_global_joint.transform.rotation.y, tfs_global_joint.transform.rotation.z, tfs_global_joint.transform.rotation.w));
        
        tf::Transform tf_joint_sensor;
        tf_joint_sensor.setOrigin(tf::Vector3(tfs_joint_sensor.transform.translation.x, tfs_joint_sensor.transform.translation.y, tfs_joint_sensor.transform.translation.z));
        tf_joint_sensor.setRotation(tf::Quaternion(tfs_joint_sensor.transform.rotation.x, tfs_joint_sensor.transform.rotation.y, tfs_joint_sensor.transform.rotation.z, tfs_joint_sensor.transform.rotation.w));
        
        // Calculate sensor boundaries based on the time elapsed
        float pan_min_local;
        float pan_max_local;
        if (req.time < 0.01)  // if no time limit
        {
          pan_min_local = -3.14;
          pan_max_local = 3.14;
          
        }
        else
        {
          pan_min_local = only2decimals(current_pan_ - pan_vel_ * req.time);
          pan_max_local = only2decimals(current_pan_ + pan_vel_ * req.time);
          if (abs(pan_min_) > 0.01 || abs(pan_max_) > 0.01)  // if not revolute joint
          {
            if (pan_min_local < pan_min_)
            {
              pan_min_local = pan_min_;
            }
            if (pan_max_local > pan_max_)
            {
              pan_max_local = pan_max_;
            }
          }
        }
        
        // TODO: Estimate the optimal pan position for the 3d reconstruction
        //       Use pan_min_local/pan_max_local to calculate all the possible poses and estimate their gains
        float max_gain = 0.0;
        int possible_pans = only2decimals(1 + (pan_max_local - pan_min_local) / pan_step_);
        ROS_INFO_STREAM("\n pan_min_: " << pan_min_ <<
                        "\n pan_max_: " << pan_max_ <<
                        "\n current_pan_: " << current_pan_ <<
                        "\n req.time: " << req.time <<
                        "\n pan_min_local: " << pan_min_local <<
                        "\n pan_max_local: " << pan_max_local <<
                        "\n pan_step: " << pan_step_ <<
                        "\n possible_pans: " << possible_pans);
        for (int i = 0; i < possible_pans; ++i)
        {
          calculated = true;
          float candidate_pan = only2decimals(pan_min_local + pan_step_ * i);
          ROS_INFO_STREAM("candidate_pan(" << candidate_pan <<
                          ") = pan_min_local(" << pan_min_local <<
                          ") + pan_step_(" << pan_step_ <<
                          ") * i(" << i << ")");
          
          tf::Quaternion quat_aux;
          
          tf::Transform tf_current_pan;
          tf_current_pan.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
          tf::Quaternion q1;
          q1.setRPY(-current_pan_, 0.0, 0.0);
          tf_current_pan.setRotation(q1);
          
          tf::Transform tf_candidate_pan;
          tf_candidate_pan.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
          tf::Quaternion q3;
          q3.setRPY(candidate_pan, 0.0, 0.0);
          tf_candidate_pan.setRotation(q3);
          
          tf::Transform tf_candidate_global_sensor;
          tf_candidate_global_sensor = tf_global_joint * tf_current_pan * tf_candidate_pan * tf_joint_sensor;
          
          octomap_msgs::EstimateGain srv_estimate_gain;
          tf::poseTFToMsg(tf_candidate_global_sensor, srv_estimate_gain.request.pose.pose);
          srv_estimate_gain.request.pose.header.stamp = ros::Time::now();
          srv_estimate_gain.request.pose.header.frame_id = global_frame_;
          srv_estimate_gain.request.h_res = h_res_;
          srv_estimate_gain.request.v_res = v_res_;
          srv_estimate_gain.request.angle_h_min = angle_h_min_;
          srv_estimate_gain.request.angle_h_max = angle_h_max_;
          srv_estimate_gain.request.angle_v_min = angle_v_min_;
          srv_estimate_gain.request.angle_v_max = angle_v_max_;
          srv_estimate_gain.request.range_min = range_min_;
          srv_estimate_gain.request.range_max = range_max_;
          srv_estimate_gain.request.unknown_gain_value = unknown_gain_value_;
          if (srvc_estimate_gain_.call(srv_estimate_gain))
          {
            ROS_INFO_STREAM("srv_estimate_gain.response.gain: " << srv_estimate_gain.response.gain << "\tmax_gain: " << max_gain);
            if (srv_estimate_gain.response.gain > max_gain)
            {
              tf_optimal_global_sensor = tf_candidate_global_sensor;
              pan_optimal = candidate_pan;
              ROS_INFO_STREAM("pan_optimal: " << pan_optimal);
              max_gain = srv_estimate_gain.response.gain;
              // TODO: if all are unknown do not find more
              //~ if (max_gain == unknown_gain_value_ * fov_rays_)
              //~ {
                //~ ROS_INFO_STREAM("break");
                //~ break;
              //~ }
            }
          }
          else
          {
            ROS_WARN_STREAM("Call to service " << srvc_estimate_gain_.getService() << " failed");
          }
        }
        ROS_INFO_STREAM("END for (int i = 0; i < possible_pans; ++i)");
        ROS_INFO_STREAM("pan_optimal: " << pan_optimal);
      }
    
    }
    
    // Move sensor (joint)
    std_msgs::Float64 pan_optimal_msg;
    pan_optimal_msg.data = pan_optimal;
    sensor_pos_pub_.publish(pan_optimal_msg);
    res.success = calculated;
    res.message = std::to_string(pan_optimal);
    tf::poseTFToMsg(tf_optimal_global_sensor, res.pose);
    res.pan = pan_optimal;
  }
  
  
  void Planner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    float old_x_ = current_goal_.pose.position.x;
    float old_y_ = current_goal_.pose.position.y;
    float new_x_ = msg->pose.position.x;
    float new_y_ = msg->pose.position.y;
    float distance = std::sqrt(std::pow(old_x_ - new_x_, 2) + std::pow(old_y_ - new_y_, 2));
    // If the new goal is not too far from the current goal
    if (distance > goal_selection_radius_)
    {
      //Cancel the current goal (if active) and send a new one
      if (goal_running_ && !move_base_ac_.getState().isDone())
      {
        ROS_INFO_STREAM("Sending cancelGoal()");
        move_base_ac_.cancelGoal();
        move_base_ac_.waitForResult(/*ros::Duration(1)*/);
      }
      // Get the pose of the robot
      geometry_msgs::TransformStamped tfs_global_base;
      geometry_msgs::TransformStamped tfs_base_sensor;
      try
      {
        tfs_global_base = tf_buffer_.lookupTransform(global_frame_, base_frame_, ros::Time(0));
        tfs_base_sensor = tf_buffer_.lookupTransform(base_frame_, sensor_frame_, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      // Fill the message with the robot pose and the goal pose
      nav_msgs::GetPlan srv_plan;
      srv_plan.request.start.header = tfs_global_base.header;
      srv_plan.request.start.pose.position.x = tfs_global_base.transform.translation.x;
      srv_plan.request.start.pose.position.y = tfs_global_base.transform.translation.y;
      srv_plan.request.start.pose.position.z = tfs_global_base.transform.translation.z;
      srv_plan.request.start.pose.orientation = tfs_global_base.transform.rotation;
      srv_plan.request.goal = *msg;
      srv_plan.request.tolerance = goal_selection_radius_;
      // Get the trajectory (list of poses) from the current pose to the goal
      if (srvc_plan_.call(srv_plan))
      {
        trajectory_msgs::JointTrajectory sensor_joint_trajectory;
//        sensor_joint_trajectory.header.frame_id = sensor_frame_;
        if (optimize_pan_)
          sensor_joint_trajectory.joint_names.push_back(pan_joint_name_);
        // Calculate camera poses based on srv_plan.response.plan
        int number_of_poses = srv_plan.response.plan.poses.size();
        float time_between_poses = (float) (costmap_resolution_ / base_max_trans_vel_);
        float calculated_last_pan = current_pan_;

        tf::Transform last_tf_global_base;
        last_tf_global_base.setOrigin(tf::Vector3(tfs_global_base.transform.translation.x, tfs_global_base.transform.translation.y, tfs_global_base.transform.translation.z));
        last_tf_global_base.setRotation(tf::Quaternion(tfs_global_base.transform.rotation.x, tfs_global_base.transform.rotation.y, tfs_global_base.transform.rotation.z, tfs_global_base.transform.rotation.w));

        tf::Transform calculated_last_tf_base_sensor;
        calculated_last_tf_base_sensor.setOrigin(tf::Vector3(tfs_base_sensor.transform.translation.x, tfs_base_sensor.transform.translation.y, tfs_base_sensor.transform.translation.z));
        calculated_last_tf_base_sensor.setRotation(tf::Quaternion(tfs_base_sensor.transform.rotation.x, tfs_base_sensor.transform.rotation.y, tfs_base_sensor.transform.rotation.z, tfs_base_sensor.transform.rotation.w));


//        bool sensor_to_left_expected = sensor_to_left_;  // test
        for (int i = 0; i < number_of_poses; ++i)
        {
          tf::Transform tf_global_base;
          tf_global_base.setOrigin(tf::Vector3(srv_plan.response.plan.poses[i].pose.position.x,
                                               srv_plan.response.plan.poses[i].pose.position.y,
                                               srv_plan.response.plan.poses[i].pose.position.z));
          tf_global_base.setRotation(tf::Quaternion(srv_plan.response.plan.poses[i].pose.orientation.x,
                                                    srv_plan.response.plan.poses[i].pose.orientation.y,
                                                    srv_plan.response.plan.poses[i].pose.orientation.z,
                                                    srv_plan.response.plan.poses[i].pose.orientation.w));
          tf::Transform tf_global_sensor = tf_global_base * calculated_last_tf_base_sensor;
          tf::Transform tf_optimal_global_base;
          // TODO: estimateOptimalPose¿?
          //~ float pan_optimal = optimizeCurrentPose(tf_global_sensor, calculated_last_pan, time_between_poses, tf_optimal_global_base);
          //~ trajectory_msgs::JointTrajectoryPoint optimal_point;
          //~ optimal_point.positions.push_back(pan_optimal);
          //~ optimal_point.time_from_start = ros::Duration(time_between_poses * i);

          //~ sensor_joint_trajectory.points.push_back(optimal_point);
        }
        ROS_INFO_STREAM("\n END for (int i = 0; i < number_of_poses; ++i)");

        sensor_joint_trajectory.header.stamp = srv_plan.response.plan.header.stamp;
        sensor_traj_pub_.publish(sensor_joint_trajectory);

        // Send goal to move_base
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = srv_plan.response.plan.header.stamp;
        move_base_goal.target_pose.header.frame_id = srv_plan.request.goal.header.frame_id;
        move_base_goal.target_pose.pose = srv_plan.request.goal.pose;
        ROS_INFO_STREAM("Sending goal...");
        move_base_ac_.sendGoal(move_base_goal,
            boost::bind(&Planner::doneCallback, this, _1, _2),
            boost::bind(&Planner::activeCallback, this),
            boost::bind(&Planner::feedbackCallback, this, _1));
      }
      else
      {
        ROS_WARN_STREAM("Call to service " << srvc_plan_.getService() << " failed");
      }
    }
  }


  void Planner::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (optimize_pan_)
    {
      for (int i = 0; i < msg->name.size(); ++i)
      {
        if (msg->name[i] == pan_joint_name_)
        {
          current_pan_ = only2decimals(msg->position[i]);
        }
      }
      // test ///////////////////////////////////////////////////////////////////////////
//      if (current_pan_ < pan_min_ * 0.9)
//      {
//        sensor_to_left_ = false;
//      }
//      if (current_pan_ > pan_max_ * 0.9)
//      {
//        sensor_to_left_ = true;
//      }
      // test ///////////////////////////////////////////////////////////////////////////
    }
  }


  // Called once when the goal becomes active
  void Planner::activeCallback()
  {
    goal_running_ = true;
    ROS_INFO_STREAM("Goal just went active");
  }

  // Called every time feedback is received for the goal
  void Planner::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    // geometry_msgs/PoseStamped base_position
  }

  // Called once when the goal completes
  void Planner::doneCallback(const actionlib::SimpleClientGoalState& state,
                       const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    goal_running_ = false;
    ROS_INFO_STREAM("Finished in state [" << state.toString() << "]");
  }

}  // namespace planner


int main(int argc, char** argv)
{
  ros::init(argc, argv, "reconstruction_3d_planner");

  planner::Planner explore;

  ros::spin();

  return 0;
}

