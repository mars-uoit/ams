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

#ifndef _QUAD_TELEOP_H_
#define _QUAD_TELEOP_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <mavros/OverrideRCIn.h>
#include <ams_uav/ams_uav.h>
#include <ams_uav/DesiredPoseChange.h>

class QuadTeleop
{
  private:

    ros::NodeHandle nh_, pnh_;
    //  Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber control_mode_sub_;
    //  Publishers
    ros::Publisher send_rc_pub_;
    // Services
    ros::ServiceClient desired_pose_srv_;
    //  Members
    std_msgs::Int32 control_mode_;
    bool is_armed_, rc_teleop_enabled_, sp_teleop_enabled_;
    int32_t roll_axes_, roll_max_, roll_min_;
    int32_t pitch_axes_, pitch_max_, pitch_min_;
    int32_t yaw_axes_, yaw_max_, yaw_min_;
    int32_t throttle_axes_, throttle_max_, throttle_min_;
    // Joystick variables for manual sp changing
    int32_t move_x_button_, move_y_button_;
    int32_t move_ccw_button_, move_cw_button_;
    int32_t height_axis_;
    double joy_height_;
    bool allow_height_change_;
    geometry_msgs::PoseStamped internal_desired_pose_;
    // Member functions
    void joyCallback(const sensor_msgs::JoyConstPtr& joy_msg);
    void controlModeCallback(const std_msgs::Int32ConstPtr& control_mode_msg);
    int32_t joyRateScale(double joy, int32_t max, int32_t min);
    int32_t joyThrottleScale(double joy, int32_t max, int32_t min);

  public:

    QuadTeleop(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~QuadTeleop();
};
#endif  // _QUAD_TELEOP_H_
