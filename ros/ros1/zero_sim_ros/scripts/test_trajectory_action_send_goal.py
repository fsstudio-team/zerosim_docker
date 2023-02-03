#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Header
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult

from trajectory_msgs.msg import JointTrajectoryPoint
import math
import numpy as np
from tf import TransformListener
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
import actionlib

class MoveRobotClass():
	def __init__(self):
		rospy.init_node('ur10_unity_action_send_goal')
		self.ur_10_pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		rospy.sleep(3)

	def moveRobot(self):

		ur_10_action_goal = FollowJointTrajectoryActionGoal()

		# HEADER
		h = Header()
		h.stamp = rospy.get_rostime()
		ur_10_action_goal.header.stamp = h.stamp

		# JOINT NAMES
		ur_10_action_goal.goal.trajectory.joint_names.append('shoulder_pan_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('shoulder_lift_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('elbow_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('wrist_1_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('wrist_2_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('wrist_3_joint')
		ur_10_action_goal.goal.trajectory.joint_names.append('finger_joint')
 
		# GOAL ID
		ur_10_action_goal.goal_id.id = 'move_group-' + str(h.stamp.nsecs)
		ur_10_action_goal.goal_id.stamp = h.stamp

		# GOAL
		# ur_10_action_goal.goal.trajectory.header.stamp = h.stamp # Nao foi configurado
		ur_10_action_goal.goal.trajectory.header.frame_id = "world"
		
		point_0 = JointTrajectoryPoint()
		point_0.positions = [0.0, 0.0, 0.0, -5.364418029785156e-06, 0.0, 0.0] 
		point_0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		point_0.accelerations = [0.0, 0.0, 2.1105342447315522e-05, 0.0, 0.0, 0.0] 
		point_0.time_from_start.nsecs = 0
		point_0.time_from_start.secs = 0
		
		point_1 = JointTrajectoryPoint()
		point_1.positions = [0.21293854720620947, -0.08941881987723746, 1.3858880410770992e-05, 0.07614334535656375, -1.3772548609954392e-05, -0.19967275860797307]
		point_1.velocities = [0.25040549920360766, -0.10515223534362586, 1.6297377403920842e-05, 0.08954722352995227, -1.6195855354652365e-05, -0.2348055692714566]
		point_1.accelerations = [0.14181081847473967, -0.05955030782452239, 9.229607320894032e-06, 0.05071280423676912, -9.172112877098737e-06, -0.1329761928820001] 
		point_1.time_from_start.nsecs = 145995388
		point_1.time_from_start.secs = 1

		point_2 = JointTrajectoryPoint()
		point_2.positions = [0.42587709441241894, -0.17883763975447492, 2.7717760821541983e-05, 0.1522920551311573, -2.7545097219908784e-05, -0.39934551721594613] 
		point_2.velocities = [0.315, -0.13227726323338243, 2.050144225491156e-05, 0.11264678891496396, -2.0373731619077766e-05, -0.295375918483196] 
		point_2.accelerations = [0.0, 0.0, 0.0, 2.0529411967989967e-17, 1.0024126937495101e-20, 1.6423529574391974e-16] 
		point_2.time_from_start.nsecs = 821990776
		point_2.time_from_start.secs = 1	
		
		point_3 = JointTrajectoryPoint()
		point_3.positions = [0.6388156416186285, -0.2682564596317124, 4.157664123231298e-05, 0.22844076490575085, -4.131764582986318e-05, -0.5990182758239192] 
		point_3.velocities = [0.315, -0.1322772632333824, 2.050144225491156e-05, 0.11264678891496395, -2.0373731619077766e-05, -0.295375918483196] 
		point_3.accelerations = [0.0, 4.1058823935979934e-17, 0.0, -4.1058823935979934e-17, -1.0024126937495101e-20, -2.463529436158796e-16]  
		point_3.time_from_start.nsecs = 497986164
		point_3.time_from_start.secs = 2
		
		point_4 = JointTrajectoryPoint()
		point_4.positions = [0.8517541888248379, -0.35767527950894984, 5.543552164308397e-05, 0.30458947468034436, -5.509019443981757e-05, -0.7986910344318923] 
		point_4.velocities = [0.315, -0.13227726323338246, 2.050144225491156e-05, 0.11264678891496396, -2.0373731619077776e-05, -0.295375918483196] 
		point_4.accelerations = [0.0, -1.6423529574391974e-16, 0.0, 1.0264705983994984e-16, -1.5036190406242652e-20, 2.463529436158796e-16] 
		point_4.time_from_start.nsecs = 173981552
		point_4.time_from_start.secs = 3
		
		# ur_10_action_goal.goal.trajectory.points.append(point_0)
		ur_10_action_goal.goal.trajectory.points.append(point_1)
		ur_10_action_goal.goal.trajectory.points.append(point_2)
		ur_10_action_goal.goal.trajectory.points.append(point_3)
		ur_10_action_goal.goal.trajectory.points.append(point_4)

		self.ur_10_pub.publish(ur_10_action_goal)
		rospy.loginfo(f"Published the command: {ur_10_action_goal}")

if __name__ == '__main__':
	try:
		ur10_control = MoveRobotClass()
		ur10_control.moveRobot()

	except rospy.ROSInterruptException:
		pass