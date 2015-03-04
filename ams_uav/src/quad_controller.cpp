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

#include "ams_uav/quad_controller.h"

QuadController::QuadController(ros::NodeHandle nh, ros::NodeHandle pnh):
  //  Members default values
  nh_(nh),
  pnh_(pnh),
  is_first_(true),
  is_enabled_(false),
  control_mode_(ams_uav::CONTROL_DISABLED)
{
  ros::NodeHandle nh_roll_param(pnh_, "roll"),nh_pitch_param(pnh_, "pitch"),
  nh_yaw_param(pnh_, "yaw"), nh_throttle_param(pnh_,"throttle");
  // Quadrotor Parameters
  throttle_max_ = 2000;
  throttle_min_ = 1200;
  rate_max_     = 2000;
  rate_mid_     = 1500;
  rate_min_     = 1000;
  tf_listener_.reset(new tf::TransformListener);
  // ROS Publishers
  pose_desired_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mocap/pose_desired", 1);
  send_rc_pub_      = nh.advertise<mavros::OverrideRCIn>("rc/override", 1);
  // ROS Subcribers
  pose_sub_ = nh.subscribe("mocap/pose", 1, &QuadController::poseCallback,this);
  // ROS Services
  mode_change_srv_         = nh.advertiseService("mode_change", &QuadController::modeChange,this);
  desired_pose_change_srv_ = nh.advertiseService("desired_pose_change", &QuadController::desiredPoseChange,this);
  // RC ROS params
  pnh_.param<int32_t>("throttle_min", throttle_min_, throttle_min_);
  pnh_.param<int32_t>("throttle_max", throttle_max_, throttle_max_);
  pnh_.param<int32_t>("rate_max",     rate_max_,     rate_max_);
  pnh_.param<int32_t>("rate_mid",     rate_mid_,     rate_mid_);
  pnh_.param<int32_t>("rate_min",     rate_min_,     rate_min_);
  // Control ROS params
  pnh.param<double>("control_freq", control_freq_, 25.0);
  // PID ROS params
  nh_roll_param.param<double>("moving_P" ,     PID_params_[ams_uav::ROLL][0],      300.0);
  nh_roll_param.param<double>("moving_I" ,     PID_params_[ams_uav::ROLL][1],       10.0);
  nh_roll_param.param<double>("moving_D" ,     PID_params_[ams_uav::ROLL][2],        5.0);
  nh_roll_param.param<double>("holding_P",     PID_params_[ams_uav::ROLL][3],      300.0);
  nh_roll_param.param<double>("holding_I",     PID_params_[ams_uav::ROLL][4],       10.0);
  nh_roll_param.param<double>("holding_D",     PID_params_[ams_uav::ROLL][5],        5.0);
  nh_roll_param.param<double>("landing_P",     PID_params_[ams_uav::ROLL][6],      300.0);
  nh_roll_param.param<double>("landing_I",     PID_params_[ams_uav::ROLL][7],        0.0);
  nh_roll_param.param<double>("landing_D",     PID_params_[ams_uav::ROLL][8],        0.0);

  nh_pitch_param.param<double>("moving_P" ,    PID_params_[ams_uav::PITCH][0],     300.0);
  nh_pitch_param.param<double>("moving_I" ,    PID_params_[ams_uav::PITCH][1],      10.0);
  nh_pitch_param.param<double>("moving_D" ,    PID_params_[ams_uav::PITCH][2],      15.0);
  nh_pitch_param.param<double>("holding_P",    PID_params_[ams_uav::PITCH][3],     300.0);
  nh_pitch_param.param<double>("holding_I",    PID_params_[ams_uav::PITCH][4],      10.0);
  nh_pitch_param.param<double>("holding_D",    PID_params_[ams_uav::PITCH][5],      15.0);
  nh_pitch_param.param<double>("landing_P",    PID_params_[ams_uav::PITCH][6],     300.0);
  nh_pitch_param.param<double>("landing_I",    PID_params_[ams_uav::PITCH][7],       0.0);
  nh_pitch_param.param<double>("landing_D",    PID_params_[ams_uav::PITCH][8],       0.0);

  nh_yaw_param.param<double>("moving_P" ,      PID_params_[ams_uav::YAW][0],        25.0);
  nh_yaw_param.param<double>("moving_I" ,      PID_params_[ams_uav::YAW][1],         0.0);
  nh_yaw_param.param<double>("moving_D" ,      PID_params_[ams_uav::YAW][2],         0.0);
  nh_yaw_param.param<double>("holding_P",      PID_params_[ams_uav::YAW][3],        80.0);
  nh_yaw_param.param<double>("holding_I",      PID_params_[ams_uav::YAW][4],         8.0);
  nh_yaw_param.param<double>("holding_D",      PID_params_[ams_uav::YAW][5],         0.5);

  nh_throttle_param.param<double>("moving_P",  PID_params_[ams_uav::THROTTLE][0],  300.0);
  nh_throttle_param.param<double>("moving_I",  PID_params_[ams_uav::THROTTLE][1],    1.5);
  nh_throttle_param.param<double>("moving_D",  PID_params_[ams_uav::THROTTLE][2],    1.0);
  nh_throttle_param.param<double>("holding_P", PID_params_[ams_uav::THROTTLE][3],  300.0);
  nh_throttle_param.param<double>("holding_I", PID_params_[ams_uav::THROTTLE][4],   15.0);
  nh_throttle_param.param<double>("holding_D", PID_params_[ams_uav::THROTTLE][5],    5.0);
  nh_throttle_param.param<double>("takeoff_P", PID_params_[ams_uav::THROTTLE][6],  900.0);
  nh_throttle_param.param<double>("takeoff_I", PID_params_[ams_uav::THROTTLE][7],  280.0);
  nh_throttle_param.param<double>("takeoff_D", PID_params_[ams_uav::THROTTLE][8],   15.0);
  nh_throttle_param.param<double>("landing_P", PID_params_[ams_uav::THROTTLE][9],   50.0);
  nh_throttle_param.param<double>("landing_I", PID_params_[ams_uav::THROTTLE][10],  50.0);
  nh_throttle_param.param<double>("landing_D", PID_params_[ams_uav::THROTTLE][11],   2.0);

  initBounds(control_data_);

  for (int i=0; i < ams_uav::NumberModes; i++)
  {
    for (int j=0; j < ams_uav::NumberChannels; j++)
    {
      if (i == ams_uav::HOVER_CONTROL || i == ams_uav::AUTO_LANDING || i == ams_uav::AUTO_TAKEOFF)
      {
        if (j == ams_uav::THROTTLE)
        {
          if (i == ams_uav::AUTO_TAKEOFF)
          {
            setGains(&PID_data_[i][j], PID_params_[j][6], PID_params_[j][7], PID_params_[j][8]);
          }
          else if (i == ams_uav::AUTO_LANDING)
          {
            setGains(&PID_data_[i][j], PID_params_[j][9], PID_params_[j][10], PID_params_[j][11]);
          }
          else
          {
            setGains(&PID_data_[i][j], PID_params_[j][3], PID_params_[j][4], PID_params_[j][5]);
          }
        }
        else
        {
          if (i == ams_uav::AUTO_LANDING)
          {
            setGains(&PID_data_[i][j], PID_params_[j][6], PID_params_[j][7], PID_params_[j][8]);
          }
          else
          {
          setGains(&PID_data_[i][j], PID_params_[j][3], PID_params_[j][4], PID_params_[j][5]);
          }
        }
      }
    }
  }

  ROS_INFO ("AMS Quad Controller Ready");
}



void QuadController::initBounds(ControlData * ctrl)
{
  for (int i = 0; i < ams_uav::NumberChannels; i++)
  {
    if (i == ams_uav::THROTTLE)
    {
      ctrl[i].rc_max     = throttle_max_;
      ctrl[i].rc_min     = throttle_min_;
      ctrl[i].start_rc   = throttle_min_;
      ctrl[i].current_rc = throttle_min_;
    }
    else
    {
      ctrl[i].rc_max     = rate_max_;
      ctrl[i].rc_min     = rate_min_;
      ctrl[i].rc_mid     = rate_mid_;
      ctrl[i].current_rc = rate_mid_;
    }
  }
}

void QuadController::rcBound(ControlData * ctrl)
{
  if (ctrl->current_rc >= ctrl->rc_max)
  {
    ctrl->current_rc = ctrl->rc_max;
  }
  else if (ctrl->current_rc <= ctrl->rc_min)
  {
    ctrl->current_rc = ctrl->rc_min;
  }
}

void QuadController::setGains(PIDData * PID,double p, double i, double d)
{
  PID->Kp = p;
  PID->Ki = i / control_freq_;
  PID->Kd = d * control_freq_;
}

/*void QuadController::gainsCallback(ams_uav::controlConfig &config, uint32_t level)
{
  setGains(control_data[PITCH].PID,config.K_x_P,config.K_x_I,config.K_x_D);
  setGains(control_data[ROLL].PID,config.K_y_P,config.K_y_I,config.K_y_D);
  setGains(control_data[THROTTLE].PID,config.K_z_P,config.K_z_I,config.K_z_D);
  setGains(control_data[YAW].PID,config.K_yaw_P,config.K_yaw_I,config.K_yaw_D);
}*/


bool QuadController::desiredPoseChange(ams_uav::DesiredPoseChange::Request  &req,
                                       ams_uav::DesiredPoseChange::Response &res)
{
  if(control_mode_ == ams_uav::HOVER_CONTROL)
  {
    desired_pose_ = req.desired_pose;
    ROS_INFO("New Desired Pose:");
    //poseInfoPrint(desired_pose.pose);
    return true;
  }
  else
  {
    return false;
  }
}

bool QuadController::modeChange(ams_uav::ModeChange::Request  &req, ams_uav::ModeChange::Response &res)
{
  // Change the control mode.
  if(req.mode == ams_uav::CONTROL_DISABLED || req.mode == ams_uav::RC_TELEOP)
  {
    is_enabled_ = false;
  }
  else
  {
    for( int i=0; i< ams_uav::NumberChannels; i++)
    {
      control_data_[i].PID = &PID_data_[req.mode][i];
    }
    resetPID(control_data_);
    desired_pose_ = req.desired_pose;
    ROS_INFO("New Desired Pose:");
    //ams_uav::poseInfoPrint(desired_pose.pose);
    old_time_ = ros::Time::now();
    is_enabled_ = true;
  }
  is_first_ = true;
  control_mode_ = req.mode;
  ROS_WARN("Mode Change");
  return true;
}

void QuadController::resetPID(ControlData *ctrl)
{
  for (int i = 0; i < ams_uav::NumberChannels; i++)
  {
    ctrl[i].previous_error = 0.0;
    ctrl[i].total_error    = 0.0;
    if (i != ams_uav::THROTTLE)
    {
      ctrl[i].current_rc = ctrl[i].rc_mid;
    }
    else
    {
      ctrl[i].start_rc = ctrl[i].current_rc;
    }
  }
}

void QuadController::doControl(ControlData * ctrl, double error, double dt, int32_t rc_start)
{
  double p_term = ctrl->PID->Kp * error;
  double i_term = ctrl->PID->Ki * (error + ctrl->total_error);
  double d_term = (ctrl->PID->Kd * (error - ctrl->previous_error) ) / dt;
  if (i_term >= (ctrl->rc_max - ctrl->rc_min))
  {
    i_term = (ctrl->rc_max - ctrl->rc_min);
  }
  else if (i_term <= (ctrl->rc_min - ctrl->rc_max))
  {
    i_term = (ctrl->rc_min - ctrl->rc_max);
  }
  else
  {
    ctrl->total_error += error;
  }
  ctrl->current_rc = (rc_start + p_term + i_term + d_term);
  ctrl->previous_error = error;
}

void QuadController::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  if (is_first_ && is_enabled_)
  {
    ROS_INFO("Control starting at Intial Pose:");
    //////////////////poseInfoPrint(*pose_msg);
    // Initalize old values
    current_time_ = ros::Time::now();
    old_time_ = ros::Time::now();
    is_first_ = false;
  }

  else if (is_enabled_)
  {
    current_time_ = ros::Time::now();
    // Compute dt
    double dt = current_time_.toSec() - old_time_.toSec();

    if (dt > (1/control_freq_))  // Slow down control.
    {
      desired_pose_.header.stamp = current_time_;
      // Used to transform the desired pose to the base_link frame aka robot's frame
      try
      {
        tf_listener_->waitForTransform("base_link", "world",current_time_, ros::Duration(0.1));
        tf_listener_->transformPose("basae_link",desired_pose_,robot_desired_pose_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        return;
      }

      // Compute yaw error
      // A lower value causes the quadrotor to rotate CCW relative to its own frame
      doControl(&control_data_[ams_uav::YAW],-tf::getYaw(robot_desired_pose_.pose.orientation),dt,control_data_[ams_uav::YAW].rc_mid);
      // Compute X error
      // A lower value causes the quadrotor to pitch forward
      doControl(&control_data_[ams_uav::PITCH],-(robot_desired_pose_.pose.position.x),dt,control_data_[ams_uav::PITCH].rc_mid);
      // Compute Y error
      // A lower value causes the quadrotor to roll left
      doControl(&control_data_[ams_uav::ROLL],-(robot_desired_pose_.pose.position.y),dt,control_data_[ams_uav::ROLL].rc_mid);
      // Compute Z error
      // A higher value causes the quadrotor to gain altitude
      doControl(&control_data_[ams_uav::THROTTLE], robot_desired_pose_.pose.position.z,dt,control_data_[ams_uav::THROTTLE].start_rc);
      mavros::OverrideRCIn send_rc_msg;

      // The angular rates are relative and APM will stabilize the craft but
      // throttle is absolute.
      // Double check bounds;
      rcBound(&control_data_[ams_uav::ROLL]);
      rcBound(&control_data_[ams_uav::PITCH]);
      rcBound(&control_data_[ams_uav::THROTTLE]);
      rcBound(&control_data_[ams_uav::YAW]);

      send_rc_msg.channels[ams_uav::ROLL]     = (int16_t)control_data_[ams_uav::ROLL].current_rc;
      send_rc_msg.channels[ams_uav::PITCH]    = (int16_t)control_data_[ams_uav::PITCH].current_rc;
      send_rc_msg.channels[ams_uav::THROTTLE] = (int16_t)control_data_[ams_uav::THROTTLE].current_rc;
      send_rc_msg.channels[ams_uav::YAW]      = (int16_t)control_data_[ams_uav::YAW].current_rc;
      send_rc_pub_.publish(send_rc_msg);

      // Keep track of old time
      old_time_ = current_time_;
      // For rviz visual
      pose_desired_pub_.publish(desired_pose_);
    }
  }
}
