#!/usr/bin/env python

import sys

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def callback(data):
    # TODO 
    goal_hand = shaking_hands_client()

def shaking_hands_client():
    rospy.loginfo('%s Ari is shaking hands' % rospy.get_time())

    client_arm = SimpleActionClient('/arm_left_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client_arm.wait_for_server(rospy.Duration(2))

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint"]
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1)

    # TODO : Find the positions rqt_joint_trajectory_controller
    # First position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(3)))

    # Second position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[1, 1, 1, 1], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(6)))

    # third position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(9)))
   
    # Send the goal and wait
    client_arm.send_goal(goal)
    return client_arm

def main():
    rospy.init_node('run_welcome_face', anonymous=True)
    rospy.loginfo('Created node run_welcome_face')

    #rospy.Subscriber('/person_detector/detections', Detections2d, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
