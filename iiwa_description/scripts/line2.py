#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from copy import deepcopy
# from lineConstraint import lineConstraint

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('manipulator')
        
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_tolerance(0.01)
        
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
        right_arm.go()
        rospy.sleep(1)

        waypoints = []
        
        star = [[-0.2853, 0.3925], [0.2853, 0.3925], [-0.1763, 0.0573], [0, 0.6], [0.1763, 0.0573], [-0.2853, 0.3925]]

        # star = [[-0.0951+0.4, 0.0309+0.4], [0.0951+0.4, 0.0309+0.4], [-0.0588+0.4, -0.0809+0.4], [0+0.4, 0.1+0.4], [0.0588+0.4, -0.0809+0.4], [-0.0951+0.4, 0.0309+0.4]]
        
        pose = right_arm.get_current_pose().pose
        pose.position.y = star[0][0]
        pose.position.x = 0.4
        pose.position.z = star[0][1]
        right_arm.set_pose_target(pose)
        right_arm.go()
        rospy.sleep(1)
        #pose = Pose()
        for i in range(6):
            pose.position.y = star[i][0]
            pose.position.x = 0.4
            pose.position.z = star[i][1]
            waypoints.append(deepcopy(pose))
        '''
        waypoints.append(deepcopy(pose))
        pose.position.x += 0.15
        waypoints.append(deepcopy(pose))
        print waypoints
        ''' 
        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.05,        # eef_step
                                    0.0,         # jump_threshold
                                    True)        # avoid_collisions
            
            # Increment the number of attempts 
            attempts += 1
            
            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            right_arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 
        '''
        with open ("/home/jyk/Desktop/data.txt",'w') as f:
            for point in plan.joint_trajectory.points:
                    f.write(str(point.positions).replace('(','').replace(')','').replace(',',' '))
                    f.write('\n')
        '''
        
        rospy.sleep(3)
        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
        right_arm.go()
        rospy.sleep(1)
              
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
