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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ams_uav/ams_uav.h>

class QuadTrajectoryPlanner
{
private:
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Subscribers
  ros::Subscriber joy_sub_;
  ros::Subscriber mocap_pose_sub_;
  ros::Subscriber mocap_target_sub_;
  // Publishers
  ros::Publisher desired_traj_pub_;
  // Members
  int activate_button_;
  bool has_desired_trajectory_;
  double clearance_height_;
  geometry_msgs::PoseStamped initial_pose_;
  geometry_msgs::PoseStamped target_pose_;
  nav_msgs::Path desired_trajectory_;

public:
  QuadTrajectoryPlanner(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),pnh_(pnh)
  {
    // ROS params
    pnh_.param("activate_button", activate_button_, 1);
    pnh_.param("clearance_height", clearance_height_, 1.0);
    // Init everything
    has_desired_trajectory_ = false;
    initSubscribers();
    initPublishers();
  }

  void initSubscribers()
  {
    // Init Subs
    joy_sub_ = nh_.subscribe("joy", 1, &QuadTrajectoryPlanner::joyCallback, this);
    mocap_pose_sub_ = nh_.subscribe("mocap/pose", 1, &QuadTrajectoryPlanner::mocapPoseCallback, this);
    mocap_target_sub_ = nh_.subscribe("target/pose", 1, &QuadTrajectoryPlanner::mocapTargetCallback, this);
  }

  void initPublishers()
  {
    // Init Pubs
    desired_traj_pub_ = nh_.advertise<nav_msgs::Path>("desired_trajectory", 1);
  }

  void computeTrajectory()
  {
    target_pose_.pose.position.y -= 0.2;
    geometry_msgs::PoseStamped temp_pose_;
    temp_pose_ = initial_pose_;
    desired_trajectory_.poses.push_back(temp_pose_);
    temp_pose_.pose.position.z = target_pose_.pose.position.z + clearance_height_;
    desired_trajectory_.poses.push_back(temp_pose_);
    temp_pose_.pose.orientation = target_pose_.pose.orientation;
    desired_trajectory_.poses.push_back(temp_pose_);
    temp_pose_.pose.position.x = target_pose_.pose.position.x;
    temp_pose_.pose.position.y = target_pose_.pose.position.y;
    desired_trajectory_.poses.push_back(temp_pose_);
    temp_pose_.pose = target_pose_.pose;
    desired_trajectory_.poses.push_back(temp_pose_);
    desired_trajectory_.header = temp_pose_.header;
    desired_trajectory_.header.frame_id = "/world";
    desired_trajectory_.header.stamp = ros::Time::now();
    desired_traj_pub_.publish(desired_trajectory_);
  }

  void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
  {
    initial_pose_ = *pose_msg;
  }

  void mocapTargetCallback(const geometry_msgs::PoseStampedConstPtr& target_msg)
  {
    target_pose_ = *target_msg;
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if (joy_msg->buttons[activate_button_] == 1 && has_desired_trajectory_ == false)
    {
      computeTrajectory();
      ROS_INFO("Computing trajectory");
      has_desired_trajectory_ = true;
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "quad_trajectory_planner");
  ros::NodeHandle nh, pnh("~");
  QuadTrajectoryPlanner quad_trajectory_planner(nh, pnh);
  ros::spin();
  return 0;
}

