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

#include "ams_uav/fly_to_target_action_server.h"

FlyToTargetServer::FlyToTargetServer(const std::string name) :
  as_(nh_, name, boost::bind(&FlyToTargetServer::executeCB, this, _1), false),
  action_name_(name)
{
  // Init variables
  control_mode_.data = ams_uav::CONTROL_DISABLED;
  start_trajectory_  = false;
  ready_for_landing_ = false;
  is_travelling_     = false;
  start_landing_     = false;
  tf_listener.reset(new tf::TransformListener);
  // Init Srvs
  desired_pose_srv_ = nh_.serviceClient<ams_uav::DesiredPoseChange>("desired_pose_change");
  mode_change_srv_  = nh_.serviceClient<ams_uav::ModeChange>("mode_change");
  ams_cmd_          = nh_.serviceClient<mavros::CommandBool>("ams/cmd/arming");
  mocap_sub_        = nh_.subscribe("mocap/pose", 1, &FlyToTargetServer::mocapPoseCallback, this);
  // Init Subs
  state_sub_        = nh_.subscribe("ams/state", 1, &FlyToTargetServer::stateCallback, this);
  // Init Pubs
  control_mode_pub_ = nh_.advertise<std_msgs::Int32>("control_mode", 1);

  as_.start();
}

void FlyToTargetServer::mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  current_pose_ = pose_msg;
}

void FlyToTargetServer::stateCallback(const mavros::StateConstPtr& state_msg)
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

void FlyToTargetServer::executeCB(const ams_uav::FlyToTargetGoalConstPtr &goal)
{
  bool success = false;

  ros::Rate r(100.0);
  
  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      // If the client has requested preempt, stop
      as_.setPreempted();
      success = false;
      break;
    }


    if (success == true)
    {
      // In this case we've finished!
      success = true;
      break;
    }
    as_.publishFeedback(feedback_);
    r.sleep();
  }

  if (success)
  {
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fly_to_target_action");

  FlyToTargetServer fly_to_target_server(ros::this_node::getName());
  ros::spin();

  return 0;
}
