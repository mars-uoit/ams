#include <ams_lwms/lwms_trajectory.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "al5d_trajectory_node");
  ros::NodeHandle nh, pnh("~");
  AL5DTrajectory ams_manager(nh, pnh);
  ros::spin();
}
