/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Tony Baltovski
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Ontario Institute of
 *     Technology nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <mavros/State.h>
#include <mavros/CommandBool.h>
#include <ams_uav/ams_uav.h>
#include <ams_uav/DesiredPoseChange.h>
#include <ams_uav/ModeChange.h>

#include "ams_uav/quad_manager.h"


QuadManager::QuadManager(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),pnh_(pnh)
{
  // Init variables
  control_mode_.data = ams_uav::CONTROL_DISABLED;
  uav_status_.data = ams_uav::LANDED;
  start_trajectory_ = false;
  ready_for_landing_ = false;
  is_travelling_ = false;
  start_landing_ = false;
  tf_listener.reset(new tf::TransformListener);
  // ROS params
  // Joystick params
  pnh_.param<int32_t>("deadman_switch", deadman_switch_, 0);
  pnh_.param<int32_t>("arm_button", arm_button_, 2);
  pnh_.param<int32_t>("disarm_button", disarm_button_, 3);

  pnh_.param<int32_t>("disable_control_button", disable_control_button_, 6);
  pnh_.param<int32_t>("rc_teleop_button", rc_teleop_button_, 7);
  pnh_.param<int32_t>("trajectory_button", trajectory_button_, 9);
  pnh_.param<int32_t>("auto_takeoff_button", auto_takeoff_button_, 10);
  pnh_.param<int32_t>("auto_landing_button", auto_landing_button_, 11);

  // System params
  pnh_.param<bool>("publish_odom",pub_odom_, true);
  pnh_.param<bool>("show_trajectory", show_trajectory_, true);
  pnh_.param<double>("position_tolerance", position_tolerance_, .10);
  pnh_.param<double>("yaw_tolerance", yaw_tolerance_, 0.0436332313);
  pnh_.param<double>("jump_distance", jump_distance_, .15);
  // Init Srvs
  desired_pose_srv_ = nh_.serviceClient<ams_uav::DesiredPoseChange>("desired_pose_change");
  mode_change_srv_  = nh_.serviceClient<ams_uav::ModeChange>("mode_change");
  ams_cmd_          = nh_.serviceClient<mavros::CommandBool>("cmd/arming");
  // Init Subs
  joy_sub_          = nh_.subscribe("joy", 5, &QuadManager::joyCallback, this);
  mocap_sub_        = nh_.subscribe("mocap/pose", 5, &QuadManager::mocapPoseCallback, this);
  state_sub_        = nh_.subscribe("ams/state", 5, &QuadManager::stateCallback,this);
  desired_traj_sub_ = nh_.subscribe("desired_trajectory", 5, &QuadManager::trajectoryCallback, this);
  // Init Timers
  mocap_timeout_ = nh_.createTimer(ros::Duration(0.25), &QuadManager::mocapTimeOutCallback, this);
  landing_timer_ = nh_.createTimer(ros::Duration(0.5), &QuadManager::landingTimerCallback, this);
  // Init Pubs
  twist_pub_        = nh_.advertise<geometry_msgs::TwistStamped>("mocap/twist", 5);
  mocap_odom_pub_   = nh_.advertise<nav_msgs::Odometry>("mocap/odom", 5);
  control_mode_pub_ = nh_.advertise<std_msgs::Int32>("control_mode", 5);
  status_pub_       = nh_.advertise<std_msgs::Int32>("uav_status", 5);
  actual_traj_pub_  = nh_.advertise<nav_msgs::Path>("actual_trajectory", 5);

  ROS_INFO("Quad Manager started");
}

QuadManager::~QuadManager()
{
  ROS_INFO ("Destroying Quad Manager");
}


bool QuadManager::withinTolerance(geometry_msgs::Pose desired, geometry_msgs::Pose actual,
                                 double position_tolerance, double yaw_tolerance)
{
  double x = fabs(desired.position.x - actual.position.x);
  double y = fabs(desired.position.y - actual.position.y);
  double z = fabs(desired.position.z - actual.position.z);

  if ( (fabs(desired.position.x - actual.position.x) <= position_tolerance)
  &&   (fabs(desired.position.y - actual.position.y) <= position_tolerance)
  &&   (fabs(desired.position.z - actual.position.z) <= position_tolerance) 
  &&   (fabs(tf::getYaw(desired.orientation) - tf::getYaw(actual.orientation)) <= yaw_tolerance) )
  {
    ROS_DEBUG("Within %f m and %f rad", position_tolerance, yaw_tolerance);
    return true;
  }
  else
  {
    return false;
  }
}


void QuadManager::stateCallback(const mavros::StateConstPtr& state_msg)
{
  if (state_msg->armed)
  {
    if (is_armed_ == false)
    {
      ROS_WARN("APM armed verified!");
    }
    is_armed_ = true;
  }
   else if (!state_msg->armed)
  {
    if (is_armed_ == true)
    {
        ROS_WARN("APM disarmed verified!");
    }
    is_armed_ = false;
  }
}

void QuadManager::getJump(geometry_msgs::PoseStamped desired_pose, geometry_msgs::PoseStamped * in_pose)
{
  double temp_x, temp_y, temp_z;
  temp_x = desired_pose.pose.position.x;  // TODO(tonybaltovski) Make this a param
  temp_y = desired_pose.pose.position.y;
  temp_z = desired_pose.pose.position.z;

  temp_x -= in_pose->pose.position.x;
  temp_y -= in_pose->pose.position.y;
  temp_z -= in_pose->pose.position.z;

  double ratio = pow(pow(temp_x, 2) + pow(temp_y, 2) + pow(temp_z, 2), .5) / jump_distance_;
  temp_x /= ratio;
  temp_y /= ratio;
  temp_z /= ratio;

  temp_x += in_pose->pose.position.x;
  temp_y += in_pose->pose.position.y;
  temp_z += in_pose->pose.position.z;

  in_pose->pose.position.x = temp_x;
  in_pose->pose.position.y = temp_y;
  in_pose->pose.position.z = temp_z;

  in_pose->pose.orientation = desired_pose.pose.orientation;

//    THIS WAS TOO SLOW
//    ros::Time current_time = ros::Time::now();
//    geometry_msgs::PoseStamped temp_pose;
//    geometry_msgs::PoseStamped out_pose;
//    desired_pose.header.stamp = current_time;
//    out_pose.header.stamp = current_time;
//    temp_pose.header.stamp = current_time;
//    temp_pose.header.frame_id = "base_link";
//    try
//    {
//      tf_listener->waitForTransform("base_link", "odom", current_time, ros::Duration(1.0));
//      tf_listener->transformPose("base_link", desired_pose, temp_pose);
//    }
//    catch (tf::TransformException ex)
//    {
//      ROS_ERROR("%s",ex.what());
//    }

//    double ratio = pow(pow(temp_pose.pose.position.x, 2) + pow(temp_pose.pose.position.y, 2) + pow(temp_pose.pose.position.z, 2), .5) / jump_distance_;
//    temp_pose.pose.position.x /= ratio;
//    temp_pose.pose.position.y /= ratio;
//    temp_pose.pose.position.z /= ratio;
//    try
//    {
//      tf_listener->waitForTransform("odom", "base_link", current_time, ros::Duration(1.0));
//      tf_listener->transformPose("odom", temp_pose, out_pose);
//    }
//    catch (tf::TransformException ex)
//    {
//      ROS_ERROR("%s",ex.what());
//    }
//     in_pose->pose = out_pose.pose;
}

void QuadManager::trajectoryCallback(const nav_msgs::PathConstPtr& path_msg)
{
  desired_trajectory_ = *path_msg;
  has_trajectory_ = true;
  ROS_INFO("Trajectory Generated");
}

void QuadManager::mocapTimeOutCallback(const ros::TimerEvent& event)
{
  ros::Time time = ros::Time::now();
  if ((time.toSec() - old_pose_.header.stamp.toSec()) > 0.5)
  {
    ROS_ERROR("MOCAP NOT SENDING DATA!");
    // Switch to landing mode when that is developed to work without mocap
  }
}

void QuadManager::mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{

    double dt = pose_msg->header.stamp.toSec() - old_pose_.header.stamp.toSec();
    //tf::Quaternion quat;
    //tf::quaternionMsgToTF(pose_msg->pose.orientation, quat);
    //double roll, pitch, yaw;
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //geometry_msgs::Vector3Stamped euler_msg;
    //euler_msg.header.stamp = pose_msg->header.stamp;
    //euler_msg.header.frame_id = "mocap_link";
    //euler_msg.vector.x = roll;
    //euler_msg.vector.y = pitch;
    //euler_msg.vector.z = yaw;

    //double vx = (pose_msg->pose.position.x - old_pose_.pose.position.x) / dt;
    //double vy = (pose_msg->pose.position.y - old_pose_.pose.position.y) / dt;
    //double vz = (pose_msg->pose.position.z - old_pose_.pose.position.z) / dt;
    //double vr = (euler_msg.vector.x - old_euler_.vector.x) / dt;
    //double vp = (euler_msg.vector.y - old_euler_.vector.y) / dt;
    //double vya = (euler_msg.vector.z - old_euler_.vector.z) / dt;
    //geometry_msgs::TwistStamped twist_msg;
    //twist_msg.header = pose_msg->header;
    //twist_msg.twist.linear.x = vx;
    //twist_msg.twist.linear.y = vy;
    //twist_msg.twist.linear.z = vz;
    //twist_msg.twist.angular.x = vx;
    //twist_msg.twist.angular.y = vy;
    //twist_msg.twist.angular.z = vz;
    //twist_pub_.publish(twist_msg);

    //nav_msgs::Odometry odom_msg;
    //odom_msg.header = pose_msg->header;
    //odom_msg.pose.pose = pose_msg->pose;
    // TODO(tonybaltovski) Make these params
    //odom_msg.pose.covariance[0]  = 0.01;  // x
    //odom_msg.pose.covariance[7]  = 0.01;  // y
    //odom_msg.pose.covariance[14] = 0.01;  // z
    //odom_msg.pose.covariance[21] = 0.01;  // roll
    //odom_msg.pose.covariance[28] = 0.01;  // pitch
    //odom_msg.pose.covariance[35] = 0.01;  // yaw
    //odom_msg.twist.twist = twist_msg.twist;
    //odom_msg.twist.covariance[0]  = 1;  // x
    //odom_msg.twist.covariance[7]  = 1;  // y
    //odom_msg.twist.covariance[14] = 1;  // z
    //odom_msg.twist.covariance[21] = 10;  // roll
    //odom_msg.twist.covariance[28] = 10;  // pitch
    //odom_msg.twist.covariance[35] = 10;  // yaw
    //mocap_odom_pub_.publish(odom_msg);

    old_pose_ = *pose_msg;
    actual_trajectory_.poses.push_back(old_pose_);
    actual_trajectory_.header.frame_id = pose_msg->header.frame_id;
    actual_trajectory_.header.stamp = ros::Time::now();
    actual_traj_pub_.publish(actual_trajectory_);
    //old_euler_ = euler_msg;

    if (has_trajectory_  && start_trajectory_)
    {
      if (control_mode_.data ==ams_uav:: CONTROL_DISABLED)
      {
        control_mode_.data = ams_uav::AUTO_TAKEOFF;
        control_mode_pub_.publish(control_mode_);
        ams_uav::ModeChange mode_srv;
        mode_srv.request.mode = ams_uav::AUTO_TAKEOFF;
        mode_srv.request.desired_pose = desired_trajectory_.poses[1];
        if(mode_change_srv_.call(mode_srv))
        {
          ROS_INFO("Changed to AUTO_TAKEOFF mode.");
        }
        else
        {
          ROS_ERROR("Could not automaticly change mode.");
        }
      }
      else if ((control_mode_.data == ams_uav::AUTO_TAKEOFF)
      && (pose_msg->pose.position.z >= desired_trajectory_.poses[1].pose.position.z))
      {
        control_mode_.data = ams_uav::HOVER_CONTROL;
        control_mode_pub_.publish(control_mode_);
        uav_status_.data = ams_uav::IN_AIR;
        status_pub_.publish(uav_status_);
        ams_uav::ModeChange mode_srv;
        mode_srv.request.mode = ams_uav::HOVER_CONTROL;
        mode_srv.request.desired_pose = desired_trajectory_.poses[2];
        if(mode_change_srv_.call(mode_srv))
        {
          ROS_INFO("Changed to HOVER_CONTROL mode.");
        }
        else
        {
          ROS_ERROR("Could not automaticly change mode.");
        }
      }
      else if(control_mode_.data == ams_uav::HOVER_CONTROL
        && (withinTolerance(desired_trajectory_.poses[2].pose, pose_msg->pose, position_tolerance_, yaw_tolerance_))
        && !ready_for_landing_ && !is_travelling_)
      {
        is_travelling_ = true;
        intermediate_desired_pose_.pose = desired_trajectory_.poses[2].pose;
        ROS_INFO("Travelling Start");
      }

      if (is_travelling_)
      {
        if(control_mode_.data == ams_uav::HOVER_CONTROL
        && (withinTolerance(desired_trajectory_.poses[3].pose, pose_msg->pose, jump_distance_, yaw_tolerance_)))
        {
          ready_for_landing_ = true;
          is_travelling_ = false;
          ams_uav::DesiredPoseChange pose_srv;
          intermediate_desired_pose_ = desired_trajectory_.poses[3];
          pose_srv.request.desired_pose = intermediate_desired_pose_;
          if(desired_pose_srv_.call(pose_srv))
          {
            ROS_INFO("Changed desired pose.");
          }
          else
          {
            ROS_ERROR("Could not change desired pose.");
          }
        }
        else if(control_mode_.data == ams_uav::HOVER_CONTROL
        && withinTolerance(intermediate_desired_pose_.pose, pose_msg->pose, position_tolerance_+.03, yaw_tolerance_))
        {
          geometry_msgs::PoseStamped temp_pose = old_pose_;
          getJump(desired_trajectory_.poses[3], &temp_pose);
          ams_uav::DesiredPoseChange pose_srv;
          intermediate_desired_pose_ = temp_pose;
          pose_srv.request.desired_pose = intermediate_desired_pose_;
          if(desired_pose_srv_.call(pose_srv))
          {
            ROS_INFO("Changed desired pose.");
          }
          else
          {
          ROS_ERROR("Could not change desired pose.");
          }
        }
        start_landing_ = true;
        landing_start_time_ = ros::Time::now();
      }
      else if(control_mode_.data == ams_uav::HOVER_CONTROL
        && (withinTolerance(desired_trajectory_.poses[3].pose, pose_msg->pose, position_tolerance_, yaw_tolerance_))
        && ready_for_landing_)
      {
        control_mode_.data = ams_uav::AUTO_LANDING;
        control_mode_pub_.publish(control_mode_);
        ams_uav::ModeChange mode_srv;
        mode_srv.request.mode = ams_uav::AUTO_LANDING;
        mode_srv.request.desired_pose = desired_trajectory_.poses[4];
        if(mode_change_srv_.call(mode_srv))
        {
          ROS_INFO("Changed to AUTO_LANDING mode.");
        }
        else
        {
          ROS_ERROR("Could not change to AUTO_LANDING mode.");
        }
      }
    }
    else
    {
      if ((control_mode_.data == ams_uav::AUTO_TAKEOFF)
      && (pose_msg->pose.position.z >= internal_desired_pose_.pose.position.z))
      {
        control_mode_.data = ams_uav::HOVER_CONTROL;
        control_mode_pub_.publish(control_mode_);
        ams_uav::ModeChange mode_srv;
        mode_srv.request.mode = ams_uav::HOVER_CONTROL;
        internal_desired_pose_ = old_pose_;
        mode_srv.request.desired_pose = internal_desired_pose_;
        if(mode_change_srv_.call(mode_srv))
        {
          ROS_INFO("Changed to HOVER_CONTROL mode.");
        }
        else
        {
          ROS_ERROR("Could not automaticly change mode.");
        }
      }
    }

  }


void QuadManager::landingTimerCallback(const ros::TimerEvent& event)
{
  ros::Time time = ros::Time::now();
  if ((time.toSec() - landing_start_time_.toSec()) > 5.0  && start_landing_)
  {
    ready_for_landing_ = true;
    ROS_INFO_ONCE("Going to land when in desired pose tolerance.");
  }
}

void QuadManager::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Deadman switch, if not held down, it won't do anything (hopefully!)
  if (joy_msg->buttons[deadman_switch_] == 1)
  {
    //Motor arming/disarming
    if (joy_msg->buttons[arm_button_] == 1)
    {
      mavros::CommandBool srv;
      srv.request.value = true;
      ROS_INFO("Trying to arm");
      if (ams_cmd_.call(srv))
      {
        ROS_WARN("Arm request complete");
      }
      else
      {
        ROS_ERROR("Arm request incomplete");
      }
    }
    else if (joy_msg->buttons[disarm_button_] == 1)
    {
      mavros::CommandBool srv;
      srv.request.value = false;
      ROS_INFO("Trying to dis-arm");
      if (ams_cmd_.call(srv))
      {
        ROS_WARN("Dis-arm request complete");
      }
      else
      {
        ROS_ERROR("Dis-arm request incomplete");
      }
    }
    // Changes between control modes
    else if (joy_msg->buttons[disable_control_button_] == 1)
    {
      control_mode_.data = ams_uav::CONTROL_DISABLED;
      control_mode_pub_.publish(control_mode_);
      ams_uav::ModeChange mode_srv;
      mode_srv.request.mode = ams_uav::CONTROL_DISABLED;
      if(mode_change_srv_.call(mode_srv))
      {
        ROS_INFO("Changed to CONTROL_DISABLED mode.");
      }
      else
      {
        ROS_ERROR("Could not change to CONTROL_DISABLED mode.");
      }
    }
    else if (joy_msg->buttons[rc_teleop_button_] == 1
    && control_mode_.data != ams_uav::RC_TELEOP)
    {
      control_mode_.data = ams_uav::RC_TELEOP;
      control_mode_pub_.publish(control_mode_);
      ams_uav::ModeChange mode_srv;
      mode_srv.request.mode = control_mode_.data;
      if(mode_change_srv_.call(mode_srv))
      {
        ROS_INFO("Changed to RC TELEOP mode.");
      }
      else
      {
        ROS_ERROR("Could not change to RC TELEOP mode.");
      }
    }
    else if (joy_msg->buttons[trajectory_button_] == 1)
    {
      ROS_INFO("Changed to trajectory mode.");
      start_trajectory_ = true;
    }
    else if (joy_msg->buttons[8] == 1
    && control_mode_.data != ams_uav::HOVER_CONTROL)
    {
      control_mode_.data = ams_uav::HOVER_CONTROL;
      control_mode_pub_.publish(control_mode_);
      ams_uav::ModeChange mode_srv;
      mode_srv.request.mode = ams_uav::HOVER_CONTROL;
      internal_desired_pose_ = old_pose_;
      mode_srv.request.desired_pose = internal_desired_pose_;
      if(mode_change_srv_.call(mode_srv))
      {
        ROS_INFO("Changed to HOVER_CONTROL mode.");
      }
      else
      {
        ROS_ERROR("Could not change HOVER_CONTROL mode.");
      }
    }
    else if (joy_msg->buttons[auto_takeoff_button_] == 1
    && control_mode_.data != ams_uav::AUTO_TAKEOFF)
    {
      control_mode_.data = ams_uav::AUTO_TAKEOFF;
      control_mode_pub_.publish(control_mode_);
      ams_uav::ModeChange mode_srv;
      mode_srv.request.mode = ams_uav::AUTO_TAKEOFF;
      internal_desired_pose_ = old_pose_;
      internal_desired_pose_.pose.position.z += 1.0;
      mode_srv.request.desired_pose = internal_desired_pose_;
      if(mode_change_srv_.call(mode_srv))
      {
        ROS_INFO("Changed to AUTO_TAKEOFF mode.");
      }
      else
      {
        ROS_ERROR("Could not change AUTO_TAKEOFF mode.");
      }
    }
    else if (joy_msg->buttons[auto_landing_button_] == 1
    && control_mode_.data != ams_uav::AUTO_LANDING)
    {
      control_mode_.data = ams_uav::AUTO_LANDING;
      control_mode_pub_.publish(control_mode_);
      ams_uav::ModeChange mode_srv;
      mode_srv.request.mode = ams_uav::AUTO_LANDING;
      internal_desired_pose_ = old_pose_;
      internal_desired_pose_.pose.position.z = 0.0;
      mode_srv.request.desired_pose = internal_desired_pose_;
      if(mode_change_srv_.call(mode_srv))
      {
        ROS_INFO("Changed to AUTO_LANDING mode.");
      }
      else
      {
        ROS_ERROR("Could not change to AUTO_LANDING mode.");
      }
    }
  }
  else if (joy_msg->buttons[deadman_switch_] == 0)
  {
    if(is_armed_ == true)
    {
      mavros::CommandBool srv;
      srv.request.value = false;
      ROS_ERROR_THROTTLE(10,"Deadman released!");
      if (ams_cmd_.call(srv))
      {
        ROS_WARN("Dis-arm request complete");
      }
      else
      {
        ROS_FATAL("Dis-arm request incomplete");
      }
    }
  }
}
