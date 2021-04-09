#!/usr/bin/env python

import sys
import rospy
import roslaunch
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from pal_detection_msgs.msg import Detections2d
from pal_interaction_msgs.msg import TtsAction, TtsGoal

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

detection_number = 0

def callback(data):
    global detection_number 

    hand_done = False
    say_done = False

    if (detection_number == 0 and len(data.detections) > 0 ):
        
        rospy.loginfo("A new person is here!")
        goal_hand = wave_hand_client()
        goal_say = say_hello()
        rospy.loginfo(goal_hand.get_state())
        rospy.loginfo(goal_say.get_state())
        try:
            #while (goal_hand.get_state() != GoalStatus.SUCCEEDED and goal_say.get_state() != GoalStatus.SUCCEEDED):
            #while (goal_hand.get_state() != GoalStatus.SUCCEEDED):
            while (hand_done and say_done):
                if (goal_hand.get_state == GoalStatus.SUCCEEDED):
                    hand_done = True
                if (goal_say.get_state == GoalStatus.SUCCEEDED):
                    say_done = True
                rospy.sleep(1)
            rospy.sleep(4)
            rospy.loginfo("Finished his actions")
            
        except rospy.ROSInterruptException:
            pass

    detection_number = len(data.detections)

def wave_hand_client():
    rospy.loginfo('%s Ari is waving his hand' % rospy.get_time())

    client_arm = SimpleActionClient('/arm_left_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client_arm.wait_for_server(rospy.Duration(2))

    #if not rospy.is_shutdown():
    index = 0

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint"]
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1)

    # First position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(3)))

    # Second position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[1, 1, 1, 1], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(6)))

    # third position
    goal.trajectory.points.append(JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0], velocities = [0.1, 0.1, 0.1, 0.1], time_from_start = rospy.Duration(9)))
   
    # Send the goal and wait
    client_arm.send_goal(goal)
    return client_arm

def say_hello():
    rospy.loginfo('%s Ari is saying hello' % rospy.get_time())

    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server(rospy.Duration(2))
    # Create a goal to say our sentence
    goal = TtsGoal()
    goal.rawtext.text = "Hello I am Ari and welcome to isep"
    goal.rawtext.lang_id = "en_GB"
    # Send the goal and wait
    client.send_goal(goal)
    return client

def main():
    rospy.init_node('run_welcome_person', anonymous=True)
    rospy.loginfo('Created node run_welcome_person')

    rospy.Subscriber('/person_detector/detections', Detections2d, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
