#!/usr/bin/env python
#encoding=utf-8

import sys
from copy import deepcopy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from lineConstraint import lineConstraint
from std_msgs.msg import String

def move_group_python_interface_tutorial():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  arm = moveit_commander.MoveGroupCommander("manipulator")

  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % arm.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  end_effector_link = arm.get_end_effector_link()
  print "============ End effector: %s" % end_effector_link

  arm.set_goal_tolerance(0.01);
    
  arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
  arm.go()
  rospy.sleep(1)
  
  star = [[-0.2853, 0.3925], [0.2853, 0.3925], [-0.1763, 0.0573], [0, 0.6], [0.1763, 0.0573], [-0.2853, 0.3925]]
  
  L = [0.257, 0.255, 0.250, 0.150, 0, 0]
  for j in range(5):
    st_pos = deepcopy(star[j])
    st_pos.insert(0, 0.4)
    ed_pos = deepcopy(star[j+1])
    ed_pos.insert(0, 0.4)
  
    lc = lineConstraint(L, st_pos, ed_pos, 0, 10, 0.01, 1e-7)
  
    poses = lc.makePoses()
    for i in range(len(poses)):
      if not rospy.is_shutdown():
          arm.set_start_state_to_current_state()
          arm.set_pose_target(poses[i])
          arm.go()

  arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
  arm.go()
  rospy.sleep(1)
  
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
