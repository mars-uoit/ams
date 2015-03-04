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

#ifndef _QUAD_MANAGER_H_
#define _QUAD_MANAGER_H_

class QuadManager
{
  private:
    // Node handle
    ros::NodeHandle nh_, pnh_;
    // Services
    ros::ServiceClient ams_cmd_;
    ros::ServiceClient desired_pose_srv_;
    ros::ServiceClient mode_change_srv_;
    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber mocap_sub_;
    ros::Subscriber desired_traj_sub_;
    // Publishers
    ros::Publisher mocap_odom_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher control_mode_pub_;
    ros::Publisher actual_traj_pub_;
    ros::Publisher status_pub_;
    // Timers
    ros::Timer mocap_timeout_;
    ros::Timer landing_timer_;
    // Control Members
    std_msgs::Int32 control_mode_, uav_status_;
    bool is_armed_;
    bool mocap_used_;
    bool pub_odom_;
    // Joystick variables for buttons
    int32_t arm_button_, disarm_button_, deadman_switch_;
    int32_t disable_control_button_, rc_teleop_button_, auto_takeoff_button_, auto_landing_button_, trajectory_button_;
    // Trajectory variables
    bool is_travelling_;
    bool has_trajectory_, show_trajectory_, start_trajectory_;
    bool ready_for_landing_, start_landing_;
    ros::Time landing_start_time_;
    double position_tolerance_, yaw_tolerance_;
    double jump_distance_;
    boost::shared_ptr<tf::TransformListener> tf_listener;
    geometry_msgs::PoseStamped old_pose_;
    geometry_msgs::Vector3Stamped old_euler_;
    nav_msgs::Path actual_trajectory_, desired_trajectory_;
    geometry_msgs::PoseStamped internal_desired_pose_;
    geometry_msgs::PoseStamped intermediate_desired_pose_;

    bool withinTolerance(geometry_msgs::Pose desired, geometry_msgs::Pose actual,
                         double position_tolerance, double yaw_tolerance);
    void getJump(geometry_msgs::PoseStamped desired_pose, geometry_msgs::PoseStamped * in_pose);
    void stateCallback(const mavros::StateConstPtr& state_msg);
    void trajectoryCallback(const nav_msgs::PathConstPtr& path_msg);
    void mocapTimeOutCallback(const ros::TimerEvent& event);
    void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void landingTimerCallback(const ros::TimerEvent& event);

  public:
    QuadManager(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~QuadManager();
};
#endif  // _QUAD_MANAGER_H_
