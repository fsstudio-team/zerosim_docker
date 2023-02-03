#!/usr/bin/env python3

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('control_msgs')

import rospy
from std_msgs.msg import String, Header
from actionlib import SimpleActionClient, SimpleActionServer
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryAction
from actionlib_msgs.msg import GoalStatus
import typing as typ

class TrajectoryExecution:
	def __init__(self):        
		traj_controller_name = '/arm_controller/follow_joint_trajectory_action'
		self.traj_action_client = SimpleActionClient(traj_controller_name, FollowJointTrajectoryAction)
		rospy.loginfo(f'Waiting for server `{traj_controller_name}`')
		self.traj_action_client.wait_for_server()
		rospy.loginfo(f'Server `{traj_controller_name}` is ready')
	
	def send_task(self):
		task = FollowJointTrajectoryGoal()
		task.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
		task.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'finger_joint']
		task.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0,0], time_from_start = rospy.Duration(0.1)))
		task.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0,0], time_from_start = rospy.Duration(2)))
		task.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-1.6,0,0,0,0], time_from_start = rospy.Duration(3)))
		self.traj_action_client.send_goal(task)

	def cancel(self):
		self.traj_action_client.cancel_all_goals()

if __name__ == '__main__':
	rospy.init_node('test_trajectory_client')

	right = TrajectoryExecution()
	right.send_task()

	#while not rospy.is_shutdown():
	#    right.update_status()
	#    left.update_status()