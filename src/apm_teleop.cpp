/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Tony Baltovski
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
#include <roscopter/RC.h>
#include <roscopter/State.h>

enum Channels
{
  ROLL,
  PITCH,
  THROTTLE,
  YAW
};

class APMTeleop
{
private:
  ros::NodeHandle nh_;
  //  Subscribers
  ros::Subscriber joy_sub_;
  ros::Subscriber apm_state_sub_;
  ros::Subscriber control_mode_sub_;
  //  Publishers
  ros::Publisher send_rc_pub_;
  //  Members
  bool is_armed_;
  bool teleop_enabled_;
  int roll_axes_;
  int roll_max_;
  int roll_min_;
  int pitch_axes_;
  int pitch_max_;
  int pitch_min_;
  int yaw_axes_;
  int yaw_max_;
  int yaw_min_;
  int throttle_axes_;
  int throttle_max_;
  int throttle_min_;

public:
  APMTeleop() :
    //  Members default values
    is_armed_(false),
    teleop_enabled_(false),
    roll_axes_(0),
    roll_max_(1600),
    roll_min_(1400),
    pitch_axes_(1),
    pitch_max_(1600),
    pitch_min_(1400),
    yaw_axes_(2),
    yaw_max_(1600),
    yaw_min_(1400),
    throttle_axes_(3),
    throttle_max_(1700),
    throttle_min_(1100)
  {
    //  ROS params TODO(tonybaltovski) finish all
    nh_.param("roll_axes", roll_axes_, roll_axes_);
    //  Init everything
    init_subscribers();
    init_publishers();
  }

  void init_subscribers()
  {
    //  Init Subs
    joy_sub_ = nh_.subscribe("joy", 10, &APMTeleop::joy_callback, this, ros::TransportHints().tcpNoDelay());
    apm_state_sub_ = nh_.subscribe("state", 10, &APMTeleop::state_callback, this, ros::TransportHints().tcpNoDelay());
    control_mode_sub_ = nh_.subscribe("control_mode", 10,
                                      &APMTeleop::control_mode_callback, this, ros::TransportHints().tcpNoDelay());
  }

  void init_publishers()
  {
    //  Init Pub
    send_rc_pub_ = nh_.advertise<roscopter::RC>("send_rc", 10);
  }

  void state_callback(const roscopter::StateConstPtr& state_msg)
  {
    //  Double-check if armed on-board
    if (state_msg->armed)
      is_armed_ = true;
    else if (!state_msg->armed)
      is_armed_ = false;
  }

  void joy_callback(const sensor_msgs::JoyConstPtr& joy_msg)
  {
    //  Send RC commands based on the joystick
    if (teleop_enabled_)
    {
      roscopter::RC send_rc_msg;
      send_rc_msg.channel[PITCH] =
                               (pitch_max_ + pitch_min_)/2 - joy_msg->axes[pitch_axes_] * (pitch_max_ - pitch_min_)/2;
      send_rc_msg.channel[ROLL] =
                               (roll_max_ + roll_min_)/2 - joy_msg->axes[roll_axes_] * (roll_max_ - roll_min_)/2;
      send_rc_msg.channel[THROTTLE] =
                               throttle_min_ + (joy_msg->axes[throttle_axes_] + 1) * (throttle_max_ - throttle_min_)/2;
      send_rc_msg.channel[YAW] =
                               (yaw_max_ + yaw_min_)/2 - joy_msg->axes[yaw_axes_] * (yaw_max_ - yaw_min_)/2;
      send_rc_pub_.publish(send_rc_msg);
    }
  }

  void control_mode_callback(const std_msgs::Int32ConstPtr& control_mode_msg)
  {
    if (control_mode_msg->data == 1)
      teleop_enabled_ = true;
    else
      teleop_enabled_ = false;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "apm_teleop");
  APMTeleop apm_teleop;
  ros::spin();
  return 0;
}

