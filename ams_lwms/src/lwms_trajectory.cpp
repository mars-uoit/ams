#include "ams_lwms/lwms_trajectory.h"

#include <ros/ros.h>

AL5DTrajectory::AL5DTrajectory( ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(pnh)
{
  pnh.param<std::string>("frame_id", frame_id_, "al5d_base_link");
  pnh.param<std::string>("joint_1", joint_1_name_, "al5d_joint_1");
  pnh.param<std::string>("joint_2", joint_2_name_, "al5d_joint_2");
  pnh.param<std::string>("joint_3", joint_3_name_, "al5d_joint_3");
  pnh.param<std::string>("joint_4", joint_4_name_, "al5d_joint_4");
  pnh.param<std::string>("gripper", gripper_name_, "al5d_gripper");
  pnh.param<double>("gripper_open", gripper_open_, 1.57);
  pnh.param<double>("gripper_closed", gripper_closed_, 0.0);

  target_sub_ = nh_.subscribe("target_pose", 1, &AL5DTrajectory::targetPoseCallback, this);
  joint_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_controller/command", 5, false);
  compute_ik_srv_ = nh_.serviceClient<ams_lwms::DoIK>("do_ik");

  joint_traj_msg_.header.frame_id = frame_id_;
  joint_traj_msg_.joint_names.resize(5);
  joint_traj_msg_.joint_names[0] = joint_1_name_;
  joint_traj_msg_.joint_names[1] = joint_2_name_;
  joint_traj_msg_.joint_names[2] = joint_3_name_;
  joint_traj_msg_.joint_names[3] = joint_4_name_;
  joint_traj_msg_.joint_names[4] = gripper_name_;
  joint_traj_msg_.points.resize(1);
  joint_traj_msg_.points[0].positions.resize(5);
}
AL5DTrajectory::~AL5DTrajectory()
{
}

void AL5DTrajectory::assembleMsg(double j1, double j2, double j3, double j4)
{
  joint_traj_msg_.points[0].positions[0] = j1;
  joint_traj_msg_.points[0].positions[1] = j2;
  joint_traj_msg_.points[0].positions[2] = j3;
  joint_traj_msg_.points[0].positions[3] = j4;
  joint_traj_msg_.header.stamp = ros::Time::now();
  joint_traj_pub_.publish(joint_traj_msg_);
}
void AL5DTrajectory::openGripper()
{
  joint_traj_msg_.points[0].positions[4] = gripper_open_;
  joint_traj_msg_.header.stamp = ros::Time::now();
  joint_traj_pub_.publish(joint_traj_msg_);
}
void AL5DTrajectory::closeGripper()
{
  joint_traj_msg_.points[0].positions[4] = gripper_closed_;
  joint_traj_msg_.header.stamp = ros::Time::now();
  joint_traj_pub_.publish(joint_traj_msg_);
}
void AL5DTrajectory::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  ams_lwms::DoIK ik_srv;
  ik_srv.request.target = *pose_msg;
  ik_srv.request.gripper_angle = 0.0;
  if (compute_ik_srv_.call(ik_srv))
  {
    ROS_INFO("IK computed");
    AL5DTrajectory::assembleMsg(ik_srv.response.j1, ik_srv.response.j2, ik_srv.response.j3, ik_srv.response.j4);
  }
  else
  {
    ROS_INFO("Pose not attainable");
  }
}
