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
#include <dynamic_reconfigure/server.h>
#include <apm/controlConfig.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>

#define RATE_MAX 1650
#define RATE_MID 1500
#define RATE_MIN 1350

#define THROTTLE_MAX 1600
#define THROTTLE_MIN 1100
enum Modes {
	CONTROL_DISABLED = 0,
	TELEOP = 1,
	HOVER_CONTROL = 2
};

enum Channels {
	ROLL,
	PITCH,
	THROTTLE,
	YAW
};


bool is_first = true;
bool is_enabled = false;

geometry_msgs::Pose initial_pose;  //initial pose
geometry_msgs::Vector3 initial_trans;  //initial_pose translation
geometry_msgs::Vector3 initial_rot;  //initial_pose rotation
float desired_alt = 0;
ros::Time old_time;
ros::Time current_time;
roscopter::RC old_send_rc_msg;

//Subscribers
ros::Subscriber pose_sub;
ros::Subscriber status_sub;
ros::Subscriber joy_sub;
ros::Subscriber	control_mode_sub;
ros::Subscriber	send_rc_sub;
ros::Subscriber	old_send_rc_sub;
//Publishers
ros::Publisher euler_desired_pub;
ros::Publisher alt_desired_pub;
ros::Publisher ctrl_pub;
ros::Publisher send_rc_pub;
ros::Publisher old_rc_pub;

//PID Parameters
double K_yaw_P = 50, K_yaw_I = 1, K_yaw_D = 25;
double K_x_P = 500, K_x_I = 5, K_x_D = 50;
double K_y_P = 500, K_y_I = 5, K_y_D = 50;
double K_z_P = 500, K_z_I = 5, K_z_D = 50;

//PID Variables
double error_yaw, previous_error_yaw, total_error_yaw;
double error_x, previous_error_x, total_error_x;
double error_y, previous_error_y, total_error_y;
double error_z, previous_error_z, total_error_z;

int throttle_bound(int rc)
{
	if(rc >= THROTTLE_MAX) // TODO Make this a ROS param
		return THROTTLE_MAX;
	else if(rc <= THROTTLE_MIN)
		return THROTTLE_MIN;
	else
		return rc;
}
int rate_bound(int rc)
{
	if(rc >= RATE_MAX)
		return RATE_MAX;
	else if(rc <= RATE_MIN)
		return RATE_MIN;
	else
		return rc;
}

void callback(apm::controlConfig &config, uint32_t level) 
{
	K_x_P = config.K_x_P;
	K_x_I = config.K_x_I;
	K_x_D = config.K_x_D;
	K_y_P = config.K_y_P;
	K_y_I = config.K_y_I;
	K_y_D = config.K_y_D;
	K_z_P = config.K_z_P;
	K_z_I = config.K_z_I;
	K_z_D = config.K_z_D;
	K_yaw_P = config.K_yaw_P;
	K_yaw_I = config.K_yaw_I;
	K_yaw_D = config.K_yaw_D;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	desired_alt = (joy_msg->axes[3] + 1) * 0.5;
}


void controlModeCallback(const std_msgs::Int32 control_mode_msg)
{
	if(control_mode_msg.data == HOVER_CONTROL)
		is_enabled = true;
	else
		is_enabled = false;
}

void poseCallback(const geometry_msgs::Pose& pose_msg)
{
	if(is_first && is_enabled)
	{
		initial_pose = pose_msg;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose_msg.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		initial_rot.x = roll;
		initial_rot.y = pitch;
		initial_rot.z = yaw;
		initial_trans.x = pose_msg.position.x;
		initial_trans.y = pose_msg.position.y;
		initial_trans.z = pose_msg.position.z;
		ROS_INFO("Hover starting Pose: X=%f,Y=%f,Z=%f Roll=%f Pitch=%f Yaw=%f", initial_trans.x, initial_trans.y, initial_trans.z,initial_rot.x, initial_rot.y, initial_rot.z);
		//Initalize old values
		current_time = ros::Time::now();
		old_time = ros::Time::now();
		previous_error_yaw = 0.0;
		total_error_yaw = 0.0;
		previous_error_x = 0.0;
		total_error_x = 0.0;
		previous_error_y = 0.0;
		total_error_y = 0.0;
		previous_error_z = 0.0;
		total_error_z  = 0.0;
		is_first = false;
	}	 

	else if(is_enabled)
	{
		current_time = ros::Time::now();
		double dt = current_time.toSec() - old_time.toSec();
		geometry_msgs::Vector3 current_rot;  
		geometry_msgs::Vector3 current_trans; 
		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose_msg.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		current_rot.x = roll;
		current_rot.y = pitch;
		current_rot.z = yaw;
		euler_desired_pub.publish(initial_rot);
		geometry_msgs::Vector3 desired_trans;
		desired_trans.x = initial_trans.x;
		desired_trans.y = initial_trans.y;
		desired_trans.z = desired_alt + initial_trans.z;
		alt_desired_pub.publish(desired_trans);
		current_trans.x = pose_msg.position.x;
		current_trans.y = pose_msg.position.y;
		current_trans.z = pose_msg.position.z;

		if (dt > 0.02)  //Slow down to 50 Hz
		{

			//Compute yaw error and PID terms
			error_yaw = (initial_rot.z - current_rot.z);
			double p_term_yaw = K_yaw_P * error_yaw;
			double i_term_yaw = K_yaw_I * (error_yaw + total_error_yaw);
			double d_term_yaw = K_yaw_D * (error_yaw - previous_error_yaw)/dt;
			if (i_term_yaw >= ((RATE_MAX - RATE_MIN)/2))
				i_term_yaw = (RATE_MAX - RATE_MIN)/2;
			else if (i_term_yaw <= ((RATE_MIN - RATE_MAX)/2))
				i_term_yaw = (RATE_MIN - RATE_MAX)/2;

			//Compute Y error and PID terms
			error_y = (initial_trans.y - current_trans.y);
			double p_term_y = K_y_P * error_y;
			double i_term_y = K_y_I * (error_y + total_error_y);
			double d_term_y = K_y_D * (error_y - previous_error_y)/dt;
			if (i_term_y >= ((RATE_MAX - RATE_MIN)/2))
				i_term_y = (RATE_MAX - RATE_MIN)/2;
			else if (i_term_y <= ((RATE_MIN - RATE_MAX)/2))
				i_term_y = (RATE_MIN - RATE_MAX)/2;

			//Compute X error and PID terms
			error_x = (initial_trans.x - current_trans.x);
			double p_term_x = K_x_P * error_x;
			double i_term_x = K_x_I * (error_x + total_error_x);
			double d_term_x = K_x_D * (error_x - previous_error_x)/dt;
			if (i_term_x >= ((RATE_MAX - RATE_MIN)/2))
				i_term_x = (RATE_MAX - RATE_MIN)/2;
			else if (i_term_x <= ((RATE_MIN - RATE_MAX)/2))
				i_term_x = (RATE_MIN - RATE_MAX)/2;

			//Compute Z error and PID terms
			error_z = (initial_trans.z + desired_alt - current_trans.z);
			double p_term_z = K_z_P * error_z;
			double i_term_z = K_z_I * (error_z + total_error_z);
			double d_term_z = K_z_D * (error_z - previous_error_z)/dt;

			if (i_term_z >= (THROTTLE_MAX - THROTTLE_MIN))
			{
				i_term_z = THROTTLE_MAX - THROTTLE_MIN;
			}
			else if (i_term_z <= (THROTTLE_MIN - THROTTLE_MIN))
			{
				i_term_z = THROTTLE_MIN - THROTTLE_MIN; //Aka ZERO
			}				
			
			roscopter::RC send_rc_msg;
			// The angular rates are relative and APM will stabilize the craft but 
			// throttle is absolute.
			send_rc_msg.channel[ROLL] = rate_bound(RATE_MID + p_term_y + i_term_y + d_term_y);	//roll
			send_rc_msg.channel[PITCH] = rate_bound(RATE_MID + p_term_x + i_term_x + d_term_x);	//pitch
			send_rc_msg.channel[THROTTLE] = throttle_bound(THROTTLE_MIN + p_term_z + i_term_z + d_term_z);//throttle
			send_rc_msg.channel[YAW] = rate_bound(RATE_MID + p_term_yaw + i_term_yaw + d_term_yaw);	//yaw
			send_rc_pub.publish(send_rc_msg);

			//Keep track of old values
			old_time = current_time;
			previous_error_yaw = error_yaw;
			total_error_yaw += error_yaw;
			previous_error_x = error_x;
			total_error_x+= error_x;
			previous_error_y = error_y;
			total_error_y += error_y;
			previous_error_z = error_z;
			total_error_z += error_z;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apm_hover_control");
	ros::NodeHandle n;

	euler_desired_pub = n.advertise<geometry_msgs::Vector3>("mocap/euler_desired", 1);
	alt_desired_pub = n.advertise<geometry_msgs::Vector3>("mocap/alt_desired", 1);
	send_rc_pub = n.advertise<roscopter::RC>("send_rc", 1);

	pose_sub = n.subscribe("mocap/pose", 1, poseCallback);
	joy_sub = n.subscribe("joy", 1, joyCallback);
	control_mode_sub = n.subscribe("control_mode",1, controlModeCallback);

	dynamic_reconfigure::Server<apm::controlConfig> server;
	dynamic_reconfigure::Server<apm::controlConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	return 0;
}
