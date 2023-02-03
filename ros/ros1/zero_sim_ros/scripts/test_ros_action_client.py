#!/usr/bin/env python3

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('control_msgs')

import rospy
from actionlib import SimpleActionClient
from actionlib.action_server import ActionServer
from control_msgs.msg import FollowJointTrajectoryAction
from actionlib_msgs.msg import GoalStatus

import time

class TrajectoryExecution(ActionServer):
	def __init__(self):        
		# ROS Action Client
		traj_controller_name = '/arm_controller/follow_joint_trajectory'
		self.traj_action_client = SimpleActionClient(traj_controller_name, FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for a response from the trajectory action server')
		
		# ROS Action Server
		motion_request_name = '/arm_controller/follow_joint_trajectory_action'
		self.action_server = ActionServer.__init__(self, motion_request_name, FollowJointTrajectoryAction, self.move_to_joints, self.cancelCallback, False)
		self.start()
		rospy.loginfo(f'Server {motion_request_name} has started.')

	def move_to_joints(self, traj_goal_gh):
		traj_goal = traj_goal_gh.get_goal()
		rospy.loginfo('Received a trajectory execution request.')
		traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
		# self.traj_action_client.send_goal(traj_goal)
		rospy.loginfo("Sending goal and waiting for the result ...")
		exec_timeout = rospy.Duration(10)
		prmpt_timeout = rospy.Duration(10)

		start = time.time()
		self.traj_action_client.send_goal_and_wait(traj_goal, exec_timeout, prmpt_timeout)
		elapsed = time.time() - start
		rospy.loginfo(f"Time elapsed: {elapsed:02f} seconds")
		
		action_state = self.traj_action_client.get_state()
		if (action_state == GoalStatus.SUCCEEDED):
			traj_goal_gh.set_accepted()
			traj_goal_gh.set_succeeded()
			rospy.loginfo('Trajectory SUCCEEDED.')
		elif (action_state == GoalStatus.ACTIVE):
			rospy.loginfo('Trajectory ACTIVE.')
		elif (action_state == GoalStatus.PENDING):
			rospy.loginfo('Trajectory PENDING.')
		elif (action_state == GoalStatus.ABORTED):
			rospy.loginfo('Trajectory ABORTED.')
			traj_goal_gh.set_aborted()
		elif (action_state == GoalStatus.REJECTED):
			rospy.loginfo('Trajectory REJECTED.')
		elif (action_state == GoalStatus.RECALLING):
			rospy.loginfo('Trajectory RECALLING.')
		elif (action_state == GoalStatus.RECALLED):
			rospy.loginfo('Trajectory RECALLED.')
		elif (action_state == GoalStatus.LOST):
			rospy.loginfo('Trajectory LOST.')
		elif (action_state == GoalStatus.PREEMPTED):
			traj_goal_gh.set_preempted()
			rospy.loginfo('Trajectory PREEMPTED.')
	
	def cancelCallback(self, traj_goal_gh):
		self.traj_action_client.cancel_goal()

if __name__ == '__main__':
	rospy.init_node('custom_trajectory_execution')

	right = TrajectoryExecution()
	rospy.spin()