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

#ifndef _QUAD_CONTROLLER_H_
#define _QUAD_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <mavros/OverrideRCIn.h>
#include <dynamic_reconfigure/server.h>
#include <ams_uav/ams_uav.h>
#include <ams_uav/controlConfig.h>
#include <ams_uav/ModeChange.h>
#include <ams_uav/DesiredPoseChange.h>

class QuadController
{

  typedef struct PIDData {
    double Kp;
    double Ki;
    double Kd;
  }
  PIDData;

  typedef struct ControlData {
    int16_t current_rc;
    int16_t start_rc;
    double previous_error;
    double total_error;
    int16_t rc_max;
    int16_t rc_min;
    int16_t rc_mid;
    PIDData *PID;
  }
  ControlData;

  private:
    ros::NodeHandle nh_, pnh_;
    bool is_first_, is_enabled_;
    int32_t control_mode_;

    // All are in odom frame
    geometry_msgs::PoseStamped desired_pose_;

    // All are in base_link frame
    geometry_msgs::PoseStamped robot_desired_pose_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;

    //Control data structs
    ControlData control_data_[ams_uav::NumberChannels]; 
    PIDData PID_data_[ams_uav::NumberModes][ams_uav::NumberChannels];

    // PID Variables
    ros::Time old_time_, current_time_;
    double control_freq_;
    double PID_params_[ams_uav::NumberChannels][12];

    // Subscribers
    ros::Subscriber pose_sub_;

    // Publishers
    ros::Publisher pose_desired_pub_;
    ros::Publisher send_rc_pub_;

    // Services
    ros::ServiceServer mode_change_srv_;
    ros::ServiceServer desired_pose_change_srv_;

    // Quadrotor RC Parameters
    int32_t throttle_max_, throttle_min_;
    int32_t rate_max_, rate_mid_, rate_min_;

    // RC functions
    void initBounds(ControlData *ctrl);
    void rcBound(ControlData * ctrl);

    // Control functions
    void setGains(PIDData * ctrl,double p, double i, double d);
    void resetPID(ControlData * ctrl);
    void doControl(ControlData * ctrl, double error, double dt, int32_t rc_start);
    // ROS functions
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    bool modeChange(ams_uav::ModeChange::Request  &req, ams_uav::ModeChange::Response &res);
    bool desiredPoseChange(ams_uav::DesiredPoseChange::Request  &req,
                           ams_uav::DesiredPoseChange::Response &res);

  public:
    QuadController(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~QuadController(void){};
};
#endif  // _QUAD_CONTROLLER_H_
