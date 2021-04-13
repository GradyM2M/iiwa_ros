#!/usr/bin/env python
import rospy, sys
import moveit_commander
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory

count = 0

def trajectoryCallback(trajectory):
        global count        
        count += 1

        if count == 2:
                points = trajectory.trajectory[0].joint_trajectory.points
                for point in points:
                        positions = point.positions
                        velocities = point.velocities
                        accelerations = point.accelerations

                        with open('position.txt', 'a+') as f:
                                for position in positions:
                                        f.write(str(position) + ' ')
                                f.write('\n')
                        with open('velocity.txt', 'a+') as f:
                                for velocity in velocities:
                                        f.write(str(velocity) + ' ')
                                f.write('\n')
                        with open('acceleration.txt', 'a+') as f:
                                for acceleration in accelerations:
                                        f.write(str(acceleration) + ' ')
                                f.write('\n')

def trajectory_output():
	rospy.init_node('trajectory_output', anonymous=True)
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, trajectoryCallback)

        rospy.spin()


if __name__ == "__main__":
	try:
                trajectory_output()
	except rospy.ROSInterruptException:	
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
