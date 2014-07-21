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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h> 
#include <roscopter/APMCommand.h>
#include <roscopter/State.h>
#include <roscopter/Status.h>
#include <roscopter/Attitude.h>
#include <roscopter/Mavlink_RAW_IMU.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

enum Commands {
	CMD_LAUNCH = 1,
	CMD_LAND = 2,
	CMD_ARM = 3,
	CMD_DISARM = 4,
	CMD_SET_STABILIZE = 5,
	CMD_SET_ALT_HOLD = 6,
	CMD_SET_AUTO = 7,
	CMD_SET_LOITER = 8,
	CMD_SET_LAND = 9,
	RETURN_RC_CONTROL = 10
};

enum Modes {
	CONTROL_DISABLED = 0,
	TELEOP = 1,
	HOVER_CONTROL = 2
};



class APMManger
{
private:
	//Node handle
	ros::NodeHandle nh_;
	//Services
	ros::ServiceClient apm_cmd_;
	//Subscribers
	ros::Subscriber joy_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber status_sub_;
	ros::Subscriber raw_imu_sub_;
	ros::Subscriber raw_attitude_sub_;
	ros::Subscriber mocap_sub_;
	//Publishers
	ros::Publisher mocap_odom_pub_;
	ros::Publisher euler_pub_;
	ros::Publisher twist_pub_;
	ros::Publisher control_mode_pub_;
	ros::Publisher imu_pub_;
	ros::Publisher mag_pub_;
	//Members
	bool is_armed_;
	bool imu_used_;
	bool gps_used_;
	bool mocap_used_;
	int arm_button_;
	int disarm_button_;
	int eStop_button_;
	int low_battery_warn_; //mV
	geometry_msgs::PoseStamped old_pose_;
	geometry_msgs::Vector3Stamped old_euler_;
	sensor_msgs::Imu pub_imu_msg_;
	roscopter::Attitude attitude_msg_;

public:
	APMManger() :
		// Members default values
		gps_used_(false),
		is_armed_(false),
		imu_used_(true),
		mocap_used_(true),
		eStop_button_(0),
		arm_button_(2),
		disarm_button_(3),
		low_battery_warn_(1150)
		{
			// ROS params
			nh_.param("eStop_button", eStop_button_, eStop_button_);
			nh_.param("arm_button", arm_button_, arm_button_);
			nh_.param("disarm_button", disarm_button_, disarm_button_);
			nh_.param("gps_used_", gps_used_, gps_used_);
			nh_.param("mocap_used_", mocap_used_, mocap_used_);
			nh_.param("low_battery_warn", low_battery_warn_, low_battery_warn_);
			// Init everything
			init_services();
			init_subscribers();
			init_publishers();
		}

	void init_services()
	{
		//Init Srvs
		apm_cmd_ = nh_.serviceClient<roscopter::APMCommand>("command");
	}

	void init_subscribers()
	{
		//Init Subs
		joy_sub_ = nh_.subscribe("joy", 1, &APMManger::joy_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());
        	state_sub_ = nh_.subscribe("state", 1, &APMManger::state_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());
        	status_sub_ = nh_.subscribe("status", 1, &APMManger::status_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());
		if(mocap_used_)
		{
            		mocap_sub_ =  nh_.subscribe("mocap/pose", 1, &APMManger::mocap_pose_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());;
		}
		if(imu_used_)
		{
            		raw_attitude_sub_ = nh_.subscribe("attitude", 1, &APMManger::attitude_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());
            		raw_imu_sub_ = nh_.subscribe("raw_imu", 1, &APMManger::raw_imu_callback,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());
		}
	}

	void init_publishers()
	{
		//Init Pubs
		if(mocap_used_)
		{
			euler_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("mocap/euler", 1);
			twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("mocap/twist", 1);
			mocap_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("mocap/odom", 1);
		}
		control_mode_pub_ = nh_.advertise<std_msgs::Int32>("control_mode", 1);
		if(imu_used_)
		{
			imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
			mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag", 1);
		}
	}

	void state_callback(const roscopter::StateConstPtr& state_msg)
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

	void status_callback(const roscopter::StatusConstPtr& status_msg)
	{
		if (status_msg->battery_voltage < low_battery_warn_)
		{
				ROS_WARN("Battery getting low! %d V", status_msg->battery_voltage);
		}
	}

	void attitude_callback(const roscopter::AttitudeConstPtr& attitude_msg)
	{
		attitude_msg_ = *attitude_msg;
	}

	void raw_imu_callback(const roscopter::Mavlink_RAW_IMUConstPtr& raw_imu_msg)
	{
		//Converting from NED to ENU
		pub_imu_msg_.header = raw_imu_msg->header;
		pub_imu_msg_.header.frame_id = "imu";
		pub_imu_msg_.linear_acceleration.x = raw_imu_msg->xacc / 1000.0 * 9.81; //m/s
		pub_imu_msg_.linear_acceleration.y =-raw_imu_msg->yacc / 1000.0 * 9.81; //m/s
		pub_imu_msg_.linear_acceleration.z =-raw_imu_msg->zacc / 1000.0 * 9.81; //m/s
		pub_imu_msg_.angular_velocity.x = raw_imu_msg->xgyro / 1000.0;  //rad/s
		pub_imu_msg_.angular_velocity.y =-raw_imu_msg->ygyro / 1000.0;  //rad/s
		pub_imu_msg_.angular_velocity.z =-raw_imu_msg->zgyro / 1000.0;  //rad/s
		pub_imu_msg_.orientation = tf::createQuaternionMsgFromRollPitchYaw(attitude_msg_.roll, -attitude_msg_.pitch, -attitude_msg_.yaw);
		imu_pub_.publish(pub_imu_msg_);

		sensor_msgs::MagneticField mag_msg;
		mag_msg.magnetic_field.x = raw_imu_msg->ymag; //Units are Guass be they are not needed in most cases. TODO Convert to Telas.
		mag_msg.magnetic_field.y = raw_imu_msg->xmag;
		mag_msg.magnetic_field.z =-raw_imu_msg->zmag;
		mag_pub_.publish(mag_msg);
				
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		tf::Quaternion q = tf::createQuaternionFromRPY(attitude_msg_.roll, -attitude_msg_.pitch, -attitude_msg_.yaw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu"));
	}


	void mocap_pose_callback(const geometry_msgs::PoseConstPtr& pose_msg)
	{
		ros::Time current_time = ros::Time::now();
		double time_difference = current_time.toSec() - old_pose_.header.stamp.toSec();

		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose_msg->orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		geometry_msgs::Vector3Stamped euler_msg;
		euler_msg.header.stamp = current_time;
		euler_msg.header.frame_id = "mocap";
		euler_msg.vector.x = roll;
		euler_msg.vector.y = pitch;
		euler_msg.vector.z = yaw;
		euler_pub_.publish(euler_msg);

		double vx = (pose_msg->position.x - old_pose_.pose.position.x)/time_difference;
		double vy = (pose_msg->position.y - old_pose_.pose.position.y)/time_difference;
		double vz = (pose_msg->position.z - old_pose_.pose.position.z)/time_difference;
		double vr = (euler_msg.vector.x - old_euler_.vector.x)/time_difference;
		double vp = (euler_msg.vector.y - old_euler_.vector.y)/time_difference;
		double vya = (euler_msg.vector.z - old_euler_.vector.z)/time_difference;
		geometry_msgs::TwistStamped twist_msg;
		twist_msg.header.stamp = current_time;
		twist_msg.header.frame_id = "mocap";
		twist_msg.twist.linear.x = vx;
		twist_msg.twist.linear.y = vy;
		twist_msg.twist.linear.z = vz;
		twist_msg.twist.angular.x = vx;
		twist_msg.twist.angular.y = vy;
		twist_msg.twist.angular.z = vz;
		twist_pub_.publish(twist_msg);
	
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "mocap";
		odom_msg.pose.pose = *pose_msg;
		odom_msg.pose.covariance[0]  = 0.01;  //x
		odom_msg.pose.covariance[7]  = 0.01;  //y
		odom_msg.pose.covariance[14] = 0.01;  //z
		odom_msg.pose.covariance[21] = 0.01;  //roll
		odom_msg.pose.covariance[28] = 0.01;  //pitch
		odom_msg.pose.covariance[35] = 0.01;  //yaw
		odom_msg.twist.twist = twist_msg.twist;
		odom_msg.twist.covariance[0]  = 1;  //x
		odom_msg.twist.covariance[7]  = 1;  //y
		odom_msg.twist.covariance[14] = 1;  //z
		odom_msg.twist.covariance[21] = 10; //roll
		odom_msg.twist.covariance[28] = 10; //pitch
		odom_msg.twist.covariance[35] = 10; //yaw
		mocap_odom_pub_.publish(odom_msg);
		

		old_pose_.header.stamp = current_time; 
		old_pose_.pose.position = pose_msg->position;
		old_pose_.pose.orientation = pose_msg->orientation;
		old_euler_ = euler_msg; 
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
	{
		//E-Stop aka disables everything
		if (joy_msg->buttons[eStop_button_] == 1)
		{
			roscopter::APMCommand srv;
			srv.request.command = 4;
			ROS_INFO("Trying to dis-arm");
			if (apm_cmd_.call(srv))
			{
				ROS_ERROR("E-Stop pressed");
			}
			else
			{
				ROS_FATAL("Cannot stop robot!");
			}	
		}
		//Motor arming/disarming
		if (joy_msg->buttons[arm_button_] == 1)
		{
			roscopter::APMCommand srv;
			srv.request.command = CMD_ARM;
			ROS_INFO("Trying to arm");
			if (apm_cmd_.call(srv))
			{
				ROS_WARN("Arm request complete");
			}
			else
			{
				ROS_ERROR("Arm request incomplete");
			}
		}
		else if (joy_msg->buttons[disarm_button_] == 1)
		{
			roscopter::APMCommand srv;
			srv.request.command = CMD_DISARM;
			ROS_INFO("Trying to dis-arm");
			if (apm_cmd_.call(srv))
			{
				ROS_WARN("Dis-arm request complete");
			}
			else
			{
				ROS_ERROR("Dis-arm request incomplete");
			}
		}

		//Changes between control modes
		else if (joy_msg->buttons[6] == 1)
		{
			std_msgs::Int32 control_mode_;
			control_mode_.data = CONTROL_DISABLED;
			control_mode_pub_.publish(control_mode_);
			ROS_WARN("Control disabled!");
		}
		else if (joy_msg->buttons[7] == 1)
		{
			std_msgs::Int32 control_mode_;
			control_mode_.data = TELEOP;
			control_mode_pub_.publish(control_mode_);
			ROS_INFO("Tele-op mode.");
		}
		else if (joy_msg->buttons[8] == 1)
		{
			std_msgs::Int32 control_mode_;
			control_mode_.data = HOVER_CONTROL;
			control_mode_pub_.publish(control_mode_);
			ROS_INFO("Hover control mode.");
		}
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "apm_manager");
	APMManger apm_manager;
	ros::spin();
	return 0;
}

