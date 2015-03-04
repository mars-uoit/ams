#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::Publisher test_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_controller/command", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    trajectory_msgs::JointTrajectory msg;
    test_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
