#!/usr/bin/env python

# Copyright 2020 PAL Robotics SL. All Rights Reserved
 
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.
#
# Author:
#   * Sammy Pfeiffer

import sys
import rospy
from actionlib import SimpleActionClient
# To get the type of msg we will need if we have the robot running:
#   rostopic type /tts/goal
#   pal_interaction_msgs/TtsActionGoal
# Action servers always have a type XXXXAction
# and the goals are always XXXXGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal

# To test your client you can use
# rosrun actionlib axserver.py /tts pal_interaction_msgs/TtsAction
# Which will trigger a little GUI faking the action server

# The goal is just a topic so you can actually just publish on it
# but it's discouraged, it would look like (remember to press TAB to get autocompletions)
# 
# rostopic pub /tts/goal pal_interaction_msgs/TtsActionGoal "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# goal_id:
#   stamp:
#     secs: 0
#     nsecs: 0
#   id: ''
# goal:
#   text:
#     section: ''
#     key: ''
#     lang_id: ''
#     arguments:
#     - section: ''
#       key: ''
#       expanded: ''
#   rawtext:
#     text: 'I like talking to people'
#     lang_id: 'en_GB'
#   speakerName: ''
#   wait_before_speaking: 0.0"

if __name__ == '__main__':
    rospy.init_node('say_something')
    # If the user adds some input, say what he wrote
    if len(sys.argv) > 1:
        text = ""
        for arg in sys.argv[1:]:
            text += arg + " "
    # If not, just say a sentence
    else:
        text = "Hello everyone, this is the final presentation of our project, i would like to show you what i am able to do today"

    rospy.loginfo("I'll say: " + text)
    # Connect to the text-to-speech action server
    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()
    # Create a goal to say our sentence
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    # Send the goal and wait
    client.send_goal_and_wait(goal)
