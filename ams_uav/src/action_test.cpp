#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ams_uav/FlyToTargetAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_test");

  actionlib::SimpleActionClient<ams_uav::FlyToTargetAction> ac("fly_to_target_action", true);  // Has it's own thread

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  ams_uav::FlyToTargetGoal goal;
  ac.sendGoal(goal);

  bool timeout = ac.waitForResult(ros::Duration(180.0));

  if (timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ac.cancelGoal();
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
