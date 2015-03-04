#!/usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from ams_lwms.srv import *

global tf_listener

def handle_do_ik(req):
  L_12 = 0.06985
  L_23 = 0.14605
  L_34 = 0.18500
  L_4G = 0.08730
  arm_base_link = 'al5d_base_link'

  traj_pose = PoseStamped()
  try:
    traj_pose = tf_listener.transformPose(arm_base_link, req.target)
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    return None

  X = traj_pose.pose.position.x
  Y = traj_pose.pose.position.y
  Z = traj_pose.pose.position.z
  alpha = req.gripper_angle

  try:
    j1 = math.atan2(Y,X)

    # Joint 2
    Xp = math.cos(j1)*X + math.sin(j1)*Y
    Zp = Z - L_12
    a2 = Xp - math.cos(alpha)*L_4G
    b2 = Zp - math.sin(alpha)*L_4G
    k2 = (math.pow(a2,2) + math.pow(b2,2) + math.pow(L_23,2) - math.pow(L_34,2))/(2*L_23)
    q2 = math.pow(a2,2) + math.pow(b2,2) - math.pow(k2,2)
    j2 = math.atan2(b2,a2) - math.atan2(math.sqrt(round(q2,6)),k2)
    j2p = math.atan2(b2,a2) + math.atan2(math.sqrt(round(q2,6)),k2)
    if j2 <= 0:
      j2 = j2p

    # Joint 3
    a3 = (a2 - math.cos(j2)*L_23)/(L_34)
    b3 = (b2 - math.sin(j2)*L_23)/(L_34)
    j23 = math.atan2(b3,a3)
    j3 = j23 - j2

    # Joint 4
    j4 = alpha - j2 - j3
    print j1, j2, j3, j4
    return DoIKResponse(j1,j2,j3,j4)
  except ValueError:
    return None

def do_ik_server():
  rospy.init_node('do_ik_server')
  global tf_listener
  tf_listener = tf.TransformListener()
  s = rospy.Service('do_ik', DoIK, handle_do_ik)
  print "Ready to do IK."
  rospy.spin()

if __name__ == "__main__":
  try:
    do_ik_server()
  except rospy.ROSInterruptException:
    pass
