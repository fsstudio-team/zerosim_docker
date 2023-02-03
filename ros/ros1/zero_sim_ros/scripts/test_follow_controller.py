#!/usr/bin/env python

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from diagnostic_msgs.msg import *

class FollowController():
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self):
        # action server
        # name = rospy.get_param('~controllers/'+name+'/action_name','follow_joint_trajectory')
        rospy.init_node('simple_action_server')

        self.server = actionlib.SimpleActionServer('follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
        rospy.loginfo("Started FollowController follow_joint_trajectory Joints: ")

    def startup(self):
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo('follow_joint_trajectory Action goal recieved.')
        traj = goal.trajectory
        rospy.loginfo("Received trajectory")
        rospy.loginfo(goal)
        rospy.loginfo("follow_joint_trajectory: Done.")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.executeTrajectory(traj):
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")




if __name__ == '__main__':
    try:
        follow_controller = FollowController()
        follow_controller.startup()

    except rospy.ROSInterruptException:
        rospy.loginfo('COULD NOT START THE ACTION SERVER')