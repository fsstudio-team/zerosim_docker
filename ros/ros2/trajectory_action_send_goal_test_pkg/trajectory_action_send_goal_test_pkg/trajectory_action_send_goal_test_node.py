#!/usr/bin/env python3

# import roslib
# roslib.load_manifest('rospy')
# roslib.load_manifest('actionlib')
# roslib.load_manifest('control_msgs')

# import rospy
# from std_msgs.msg import String, Header
# from actionlib import SimpleActionClient, SimpleActionServer
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import JointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal
# from control_msgs.msg import FollowJointTrajectoryAction
# from actionlib_msgs.msg import GoalStatus
# import typing as typ

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# from rclpy.duration import Duration
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryExecution(Node):
	def __init__(self):       
		super().__init__('trajectory_ros2_action_client') 

		traj_controller_name = '/arm_controller/follow_joint_trajectory_action'
		self._action_client = ActionClient(self, FollowJointTrajectory, traj_controller_name)
		self._action_client.wait_for_server()
		self.get_logger().info(f'Server `{traj_controller_name}` is ready')

	def send_goal(self):
		goal_msg = FollowJointTrajectory.Goal()
		# goal_msg.order = order
		# now = rclpy.time.Time().to_msg() # .to_msg transforms from <class 'rclpy.time.Time'> to <class 'builtin_interfaces.msg._time.Time'> (Duration type)
		now = self.get_clock().now().to_msg() # .to_msg transforms from <class 'rclpy.time.Time'> to <class 'builtin_interfaces.msg._time.Time'> (Duration type)
		duration = Duration(sec=1)
		now.sec += duration.sec
		now.nanosec += duration.nanosec
		
		goal_msg.trajectory.header.stamp = now
		goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'finger_joint']
		goal_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0,0], time_from_start = Duration(sec=0, nanosec=100)))
		goal_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-2.1,0,0,0,0], time_from_start = Duration(sec=2)))
		goal_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0,0,-1.6,0,0,0,0], time_from_start = Duration(sec=3)))

		self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected :(')
			return

		self.get_logger().info('Goal accepted :)')

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)
	
	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info('Result: {0}'.format(result.sequence))
		rclpy.shutdown()
		
	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
		
def main(args=None):
	rclpy.init(args=args)
	traj_exec = TrajectoryExecution()
	traj_exec.send_goal()

if __name__ == '__main__':
	main()