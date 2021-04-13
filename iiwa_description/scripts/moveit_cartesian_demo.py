#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
                        
        # Connect to the right_arm move group
        right_arm = MoveGroupCommander('manipulator')
        
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame('iiwa_link_0')
                
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_tolerance(0.001)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
                                        
        # Start in the "straight_forward" configuration stored in the SRDF file
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
        
        # Plan and execute a trajectory to the goal configuration
        right_arm.go()
        
        # Get the current pose so we can add it as a waypoint
        start_pose = Pose()
        start_pose.orientation.x = 0
        start_pose.orientation.y = 0
        start_pose.orientation.z = 0
        start_pose.orientation.w = 1
        #start_pose = right_arm.get_current_pose(end_effector_link).pose
        start_pose.position.x = 0.4
        start_pose.position.y = 0
        start_pose.position.z = 0.4
        right_arm.set_pose_target(start_pose)
        right_arm.go()
        
        
        # Initialize the waypoints list
        waypoints = []
                
        # Append the pose to the waypoints list
        
        waypoints.append(start_pose)
            
                
        # Set the next waypoint back 0.2 meters and right 0.2 meters
        wpose = deepcopy(start_pose)
        wpose.position.x += 0.15
        waypoints.append(deepcopy(wpose))

        wpose.position.z += 0.15
        waypoints.append(deepcopy(wpose))
    
        waypoints.append(deepcopy(start_pose))
    
        fraction = 0.0
        maxtries = 50
        attempts = 0
        
        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()
     
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
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

        # Move normally back to the 'resting' position
        right_arm.set_joint_value_target([0, 0, 0, 0, 0, 0, 0])
        right_arm.go()
        rospy.sleep(1)
        
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
