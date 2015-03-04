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

#ifndef _FLY_TO_TARGET_ACTION_SERVER_H_
#define _FLY_TO_TARGET_ACTION_SERVER_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <mavros/State.h>
#include <mavros/CommandBool.h>
//#include <ams/ams_common.h>
#include <ams_uav/ams_uav.h>
#include <ams_uav/DesiredPoseChange.h>
#include <ams_uav/ModeChange.h>
#include <ams_uav/FlyToTargetAction.h>

class FlyToTargetServer
{

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ams_uav::FlyToTargetAction> as_;
    std::string action_name_;
    ams_uav::FlyToTargetFeedback     feedback_;
    ams_uav::FlyToTargetResult       result_;
    // Services
    ros::ServiceClient ams_cmd_;
    ros::ServiceClient desired_pose_srv_;
    ros::ServiceClient mode_change_srv_;
    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber mocap_sub_;
    ros::Subscriber desired_traj_sub_;  // Is this needed?
    // Publishers
    ros::Publisher control_mode_pub_;
    ros::Publisher actual_traj_pub_;
    ros::Publisher status_pub_;
    // Control Members
    std_msgs::Int32 control_mode_, uav_status_;
    bool is_armed_;
    // Trajectory variables
    bool is_travelling_;
    bool has_trajectory_, show_trajectory_, start_trajectory_;
    bool ready_for_landing_, start_landing_;
    ros::Time landing_start_time_;
    double position_tolerance_, yaw_tolerance_;
    double jump_distance_;
    boost::shared_ptr<tf::TransformListener> tf_listener;
    geometry_msgs::PoseStampedConstPtr old_pose_;
    geometry_msgs::PoseStampedConstPtr current_pose_;

    bool withinTolerance(geometry_msgs::Pose desired, geometry_msgs::Pose actual,
                         double position_tolerance, double yaw_tolerance);
    void getJump(geometry_msgs::PoseStamped desired_pose, geometry_msgs::PoseStamped * in_pose);
    void stateCallback(const mavros::StateConstPtr& state_msg);
    void trajectoryCallback(const nav_msgs::PathConstPtr& path_msg);
    void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    void executeCB(const ams_uav::FlyToTargetGoalConstPtr &goal);

  public:
    FlyToTargetServer(const std::string name);
    virtual ~FlyToTargetServer(void){};

};

#endif  // _FLY_TO_TARGET_ACTION_SERVER_H_
