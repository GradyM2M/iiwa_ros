#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'manipulator'

REFERENCE_FRAME = 'iiwa_link_0'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        robot = RobotCommander()

        # Connect to the right_arm move group
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
                                
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
                
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_tolerance(0.001)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Start in the "resting" configuration stored in the SRDF file
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
         
        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        rospy.sleep(1)
        
        # Set an initial target pose with the arm up and to the right
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.x = 0.670820393249937
        target_pose.pose.orientation.y = 0.223606797749979
        target_pose.pose.orientation.z = 0.223606797749979
        target_pose.pose.orientation.w = 0.670820393249937

        # Set the start state and target pose, then plan and execute
        right_arm.set_pose_target(target_pose, end_effector_link)
        right_arm.go()
        rospy.sleep(2)
        
        
        # Store the current pose
        start_pose = right_arm.get_current_pose(end_effector_link)
        
        # Create a contraints list and give it a name
        constraints = Constraints()
        constraints.name = "Keep gripper horizontal"
        
        # Create an orientation constraint for the right gripper 
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = start_pose.header
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.absolute_x_axis_tolerance = 0.00001
        orientation_constraint.absolute_y_axis_tolerance = 3.14
        orientation_constraint.absolute_z_axis_tolerance = 0.00001
        orientation_constraint.weight = 1.0
        orientation_constraint.orientation = start_pose.pose.orientation   
        # Append the constraint to the list of contraints
        constraints.orientation_constraints.append(orientation_constraint)
          
        # Set the path constraints on the right_arm
        right_arm.set_path_constraints(constraints)
        
        # Set a target pose for the arm        
        target_pose = deepcopy(start_pose) 
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = -0.3
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.x = 0.670820393249937 
        target_pose.pose.orientation.y = -0.223606797749979
        target_pose.pose.orientation.z = -0.223606797749979
        target_pose.pose.orientation.w = 0.670820393249937
        #target_pose.pose.orientation.x = 0.707107
        #target_pose.pose.orientation.y = 0
        #target_pose.pose.orientation.z = 0
        #target_pose.pose.orientation.w = 0.707107
        

        # Set the start state and target pose, then plan and execute
        right_arm.set_start_state_to_current_state()
        right_arm.set_pose_target(target_pose, end_effector_link)
        right_arm.go()
        rospy.sleep(1)
          
        # Clear all path constraints
        right_arm.clear_path_constraints()
        
        
        # Return to the "resting" configuration stored in the SRDF file
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])

        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

        def getOrientation(self,xyz):
            pass

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
    
    
    
