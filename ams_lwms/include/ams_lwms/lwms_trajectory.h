#ifndef _AL5D_TRAJECTORY_H_
#define _AL5D_TRAJECTORY_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include <ams_lwms/DoIK.h>

class AL5DTrajectory
{
private:
  ros::NodeHandle nh_, pnh_;

  std::string frame_id_;
  std::string joint_1_name_;
  std::string joint_2_name_;
  std::string joint_3_name_;
  std::string joint_4_name_;
  std::string gripper_name_;
  double gripper_open_, gripper_closed_;

  ros::Subscriber target_sub_;

  ros::ServiceClient compute_ik_srv_;

  ros::Publisher joint_traj_pub_;
  trajectory_msgs::JointTrajectory joint_traj_msg_;
  void assembleMsg(double j1, double j2, double j3, double j4);
  void openGripper();
  void closeGripper();
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
public:
  AL5DTrajectory(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~AL5DTrajectory();

};
#endif  // _AL5D_TRAJECTORY_H_
