#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

actionName = '/arm/joint_trajectory_controller/follow_joint_trajectory/goal'

def send_joint_angles(joint_angles, client, durationSeconds):
    # Create joint trajectory message
    joint_traj = JointTrajectory()
    joint_traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint']
    
    # Create joint trajectory point message
    traj_point = JointTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(durationSeconds)
    traj_point.positions = joint_angles
    joint_traj.points.append(traj_point)
    
    # Create action goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = joint_traj
    
    # Send action goal
    client.send_goal(goal)
    # Wait for action to complete
    client.wait_for_result()

def main():
    # Initialize ROS node
    rospy.init_node('arm_controller')

    # Set joint angles
    joint_angles_1 = [0.0, 0.0, 0, 0, 0, 0]
    joint_angles_2 = [0.1, 0.1, 0, 0, 0, 0]
    joint_angles_3 = [0.3, 0.0, 0, 0, 0, 0]
    joint_angles_4 = [0.4, 0.0, 0, 0, 0, 0]
    
    # Create action client
    client = actionlib.SimpleActionClient(actionName, FollowJointTrajectoryAction)
    
    # Wait for action server to start
    client.wait_for_server()
    
    # Send joint angles
    send_joint_angles(joint_angles_1, client, 3.0)
    send_joint_angles(joint_angles_2, client, 3.0)
    send_joint_angles(joint_angles_3, client, 3.0)
    send_joint_angles(joint_angles_4, client, 3.0)

if __name__ == '__main__':
    main()
