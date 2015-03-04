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

#include "ams_uav/quad_teleop.h"

QuadTeleop::QuadTeleop(ros::NodeHandle nh, ros::NodeHandle pnh):
  //  Members default values
  nh_(nh),
  pnh_(pnh),
  is_armed_(false),
  rc_teleop_enabled_(false),
  sp_teleop_enabled_(false)
{
  joy_height_ = 0.0;
  allow_height_change_ = false;
  //  Init Subs
  joy_sub_          = nh_.subscribe("joy", 10, &QuadTeleop::joyCallback, this);
  control_mode_sub_ = nh_.subscribe("control_mode", 10, &QuadTeleop::controlModeCallback, this);
  //  Init Pub
  send_rc_pub_ = nh_.advertise<mavros::OverrideRCIn>("rc/override", 10);
  // Init Params
  pnh_.param<int32_t>("roll_axes",       roll_axes_,     0   );
  pnh_.param<int32_t>("roll_min",        roll_min_,      1000);
  pnh_.param<int32_t>("roll_max",        roll_max_,      2000);
  pnh_.param<int32_t>("pitch_axes",      pitch_axes_,    1   );
  pnh_.param<int32_t>("pitch_min",       pitch_min_,     1000);
  pnh_.param<int32_t>("pitch_max",       pitch_max_,     2000);
  pnh_.param<int32_t>("yaw_axes",        yaw_axes_,      2   );
  pnh_.param<int32_t>("yaw_min",         yaw_min_,       1000);
  pnh_.param<int32_t>("yaw_max",         yaw_max_,       2000);
  pnh_.param<int32_t>("throttle_axes",   throttle_axes_, 3   );
  pnh_.param<int32_t>("throttle_min",    throttle_min_,  1000);
  pnh_.param<int32_t>("throttle_max",    throttle_max_,  2000);

  pnh_.param<int32_t>("move_ccw_button", move_ccw_button_, 4 );
  pnh_.param<int32_t>("move_cw_button",  move_cw_button_,  5 );
  pnh_.param<int32_t>("move_x_button",   move_x_button_,   5 );  // XY are axes technically
  pnh_.param<int32_t>("move_y_button",   move_y_button_,   4 );
  pnh_.param<int32_t>("height_axis",     height_axis_,     3 );
  
  ROS_INFO ("Quad Teleop Ready");
}

QuadTeleop::~QuadTeleop(void)
{
}

void QuadTeleop::joyCallback(const sensor_msgs::JoyConstPtr& joy_msg)
{
  //  Send RC commands based on the joystick
  if (rc_teleop_enabled_)
  {
    mavros::OverrideRCIn send_rc_msg;
    send_rc_msg.channels[ams_uav::PITCH]    = joyRateScale(joy_msg->axes[pitch_axes_], pitch_max_, pitch_min_);
    send_rc_msg.channels[ams_uav::ROLL]     = joyRateScale(joy_msg->axes[roll_axes_], roll_max_, roll_min_);
    send_rc_msg.channels[ams_uav::THROTTLE] = joyThrottleScale(joy_msg->axes[throttle_axes_], throttle_max_, throttle_min_);
    send_rc_msg.channels[ams_uav::YAW]      = joyRateScale(joy_msg->axes[yaw_axes_], yaw_max_, yaw_min_);
    send_rc_pub_.publish(send_rc_msg);
  }
  else if (sp_teleop_enabled_)
  {
    // Changing the desired pose which should only be allowed in hover mode
    if(joy_msg->axes[move_x_button_] != 0 && control_mode_.data == ams_uav::HOVER_CONTROL)
    {
      ams_uav::DesiredPoseChange pose_srv;
      internal_desired_pose_.pose.position.x += joy_msg->axes[move_x_button_] * 0.1;
      pose_srv.request.desired_pose = internal_desired_pose_;
      if(desired_pose_srv_.call(pose_srv))
      {
        ROS_INFO("Changed desired pose.");
      }
      else
      {
        ROS_ERROR("Could not change desired pose.");
      }
    }
    else if(joy_msg->axes[move_y_button_] != 0 && control_mode_.data == ams_uav::HOVER_CONTROL)
    {
      ams_uav::DesiredPoseChange pose_srv;
      internal_desired_pose_.pose.position.y += joy_msg->axes[move_y_button_] * 0.1;
      pose_srv.request.desired_pose = internal_desired_pose_;
      if(desired_pose_srv_.call(pose_srv))
      {
        ROS_INFO("Changed desired pose.");
      }
      else
      {
        ROS_ERROR("Could not change desired pose.");
      }
    }
    else if(joy_msg->buttons[move_ccw_button_] != 0 && control_mode_.data == ams_uav::HOVER_CONTROL)
    {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(internal_desired_pose_.pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      yaw -= 0.1745;
      internal_desired_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      ams_uav::DesiredPoseChange pose_srv;
      pose_srv.request.desired_pose = internal_desired_pose_;
      if(desired_pose_srv_.call(pose_srv))
      {
        ROS_INFO("Changed desired pose.");
      }
      else
      {
        ROS_ERROR("Could not change desired pose.");
      }
    }
    else if(joy_msg->buttons[move_cw_button_] != 0 && control_mode_.data == ams_uav::HOVER_CONTROL)
    {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(internal_desired_pose_.pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      yaw += 0.1745;
      internal_desired_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      ams_uav::DesiredPoseChange pose_srv;
      pose_srv.request.desired_pose = internal_desired_pose_;
      if(desired_pose_srv_.call(pose_srv))
      {
        ROS_INFO("Changed desired pose.");
      }
      else
      {
        ROS_ERROR("Could not change desired pose.");
      }
    }
    else if (control_mode_.data == ams_uav::HOVER_CONTROL && (fabs(joy_msg->axes[height_axis_] - joy_height_) > 0.05))
    {
      if (allow_height_change_)
      {
        ams_uav::DesiredPoseChange pose_srv;
        pose_srv.request.desired_pose = internal_desired_pose_;
        pose_srv.request.desired_pose.pose.position.z = internal_desired_pose_.pose.position.z
        + joy_msg->axes[height_axis_] * 0.5;  // only change every 5cm, 50cm 
        if(desired_pose_srv_.call(pose_srv))
        {
          ROS_INFO("Changed desired pose.");
        }
        else
        {
          ROS_ERROR("Could not change desired pose.");
        }
        joy_height_ = joy_msg->axes[height_axis_];
      }
      else
      {
        if (fabs(joy_msg->axes[height_axis_] - joy_height_) < 0.05)
        {
          allow_height_change_ = true;
          ROS_INFO("Height change allowed");
          joy_height_ = joy_msg->axes[height_axis_];
        }
        else
        {
          ROS_INFO_ONCE("Move height axis to %f", joy_height_);
        }
      }
    }
  }
}

void QuadTeleop::controlModeCallback(const std_msgs::Int32ConstPtr& control_mode_msg)
{
  if (control_mode_msg->data == ams_uav::RC_TELEOP)
  {
    rc_teleop_enabled_ = true;
  }
  else if (control_mode_msg->data == ams_uav::SP_TELEOP)
  {
    sp_teleop_enabled_ = true;
  }
  else
  {
    rc_teleop_enabled_ = false;
    sp_teleop_enabled_ = false;
  }
}

int32_t QuadTeleop::joyRateScale(double joy, int32_t max, int32_t min)
{
  return (max + min)/2 - joy * (max - min)/2;
}

int32_t QuadTeleop::joyThrottleScale(double joy, int32_t max, int32_t min)
{
  return min + (joy + 1) * (max - min)/2;
}

