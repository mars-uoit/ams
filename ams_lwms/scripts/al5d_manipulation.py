#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from ams_lwms.srv import *
from sensor_msgs.msg import Joy

class AL5DTrajectory:
  def __init__(self):
    rospy.init_node("al5d_manipulation_node", anonymous=False)
    self.frame_id = rospy.get_param("frame_id", "al5d_base_link")
    self.joint_1_name = rospy.get_param("joint_1", "al5d_joint_1")
    self.joint_2_name = rospy.get_param("joint_2", "al5d_joint_2")
    self.joint_3_name = rospy.get_param("joint_3", "al5d_joint_3")
    self.joint_4_name = rospy.get_param("joint_4", "al5d_joint_4")
    self.gripper_name = rospy.get_param("gripper", "al5d_gripper")
    self.gripper_open = rospy.get_param("gripper_open", -1.57)
    self.gripper_closed = rospy.get_param("gripper_closed", 0.0)
    self.joints = [self.joint_1_name, self.joint_2_name, self.joint_3_name, self.joint_4_name, self.gripper_name]
    self.tucked = [0.0, 2.70, -2.50, 0.0, 0.0]
    self.ready =  [0.0, 1.57, -1.57, 0.0, 0.0]
    self.angles = [0,0,0,0,0]
    self.traj = JointTrajectory()
    self.trajPoint = JointTrajectoryPoint()
    self.traj.joint_names = self.joints
    self.pick_target = PoseStamped()
    self.place_target = PoseStamped()
    ## Initial joint angles
    self.trajPoint.positions = self.tucked
    self.trajPoint.velocities = [0.0 for i in self.traj.joint_names]
    self.trajPoint.accelerations = [0.0 for i in self.traj.joint_names]
    self.traj.points.append(self.trajPoint)
    self.traj.header.frame_id = self.frame_id

    rospy.wait_for_service("do_ik", 3)
    self.doIKService=rospy.ServiceProxy("do_ik", DoIK)
    self.joint_traj_pub = rospy.Publisher('joint_controller/command', JointTrajectory, queue_size = 10)
    #self.pick_pose_sub = rospy.Subscriber("target/pose", PoseStamped, self.pickPoseCallback)
    self.place_pose_sub = rospy.Subscriber("target/pose", PoseStamped, self.placePoseCallback)
    self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
    #self.manipulator_srv = rospy.Service('do_manipulation', Manipulation, self.manipulatorService)
    rospy.spin()

  def manipulatorService(self):
    print "hi"

  def joyCallback(self, data):
    if data.buttons[7] == 1:
      try:
        response = self.doIKService(self.pick_target,0)
        self.doPick(data, response)
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    elif data.buttons[8] == 1:
      try:
        response = self.doIKService(self.place_target,0)
        self.doPlace(data, response)
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def pickPoseCallback(self, data):
    self.pick_target = data

  def placePoseCallback(self, data):
    self.place_target = data

  def publishData(self, angles):
    self.trajPoint.positions = angles
    self.traj.points.append(self.trajPoint)
    self.traj.header.stamp = rospy.Time.now()
    self.trajPoint.time_from_start = rospy.Duration(2.0)
    self.joint_traj_pub.publish(self.traj)
    print angles
    rospy.sleep(3)
    return angles

  def actionGripper(self, data):
    self.trajPoint.positions.pop()
    self.trajPoint.positions.append(data)

  def doPick(self, data, response):
    self.fromTuck()
    print "Moving to target"
    new_angles = self.angles
    new_angles[0] = response.j1
    self.joints = self.publishData(new_angles)
    new_angles = self.angles
    print "Gripper Open"
    new_angles[4] = self.gripper_open
    self.angles = self.publishData(new_angles)
    self.angles = self.publishData([response.j1, response.j2, response.j3, response.j4, self.angles[4]])
    new_angles = self.angles
    print "Gripper Close"
    new_angles[4] = self.gripper_closed
    self.angles = self.publishData(new_angles)
    self.toTuck()

  def doPlace(self, data, response):
    self.fromTuck()
    print "Moving to target"
    new_angles = self.angles
    new_angles[0] = response.j1
    self.joints = self.publishData(new_angles)
    new_angles = self.angles
    self.angles = self.publishData([response.j1, response.j2, response.j3, response.j4, self.angles[4]])
    new_angles = self.angles
    print "Gripper Open"
    new_angles[4] = self.gripper_open
    self.angles = self.publishData(new_angles)
    self.toTuck()


  def openGripper(self):
    print "Open"
    self.actionGripper(self.gripper_open)
    self.traj.header.stamp = rospy.Time.now()
    self.trajPoint.time_from_start = rospy.Duration(2.0)
    self.joint_traj_pub.publish(self.traj)
    rospy.sleep(3)

  def closeGripper(self):
    print "Close"
    self.actionGripper(self.gripper_closed)
    self.traj.header.stamp = rospy.Time.now()
    self.trajPoint.time_from_start = rospy.Duration(2.0)
    self.joint_traj_pub.publish(self.traj)
    rospy.sleep(3)


  def fromTuck(self):
    #  This is a tuck
    print "Starting manpulation"
    self.angles = self.publishData(self.tucked)
    #  This is ready
    print "Ready to manpulate"
    self.angles = self.publishData(self.ready)

  def toTuck(self):
    new_angles = self.angles
    temp = new_angles[0]
    new_angles = self.ready
    new_angles[0] = temp
    new_angles[4] = self.angles[4]
    self.angles = self.publishData(new_angles)
    print "Back to ready"
    new_angles = self.angles
    new_angles[0] = 0.0
    self.angles = self.publishData(new_angles)
    print "Back to tucked"
    self.angles = self.publishData(self.tucked)
    print "Done"

if __name__ == '__main__':
  try:
    AL5DTrajectory()
  except rospy.ROSInterruptException:
    pass
