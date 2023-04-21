#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal, Input
from pal_web_msgs.msg import WebGoTo

class RobotBase(object):

    def __init__(self):
        rospy.loginfo("RobotBase::init node")

        # Connect to the text-to-speech action server
        self.tts = SimpleActionClient('/tts', TtsAction)
        self.tts.wait_for_server()
        rospy.loginfo("RobotBase::Connected to TTS action client")

        # Create a publisher to publish web pages
        self.web_publisher = rospy.Publisher(
            "/web/go_to", WebGoTo, queue_size=10)
        rospy.sleep(0.5) # allow some time to properly connect the publisher
        rospy.loginfo("RobotBase::Publishing in: " +
            self.web_publisher.resolved_name)

        # Create a subscriber to the touch screen inputs
        self.user_input_sub = rospy.Subscriber(
            '/user_input', Input, self.user_input_touch_cb)
        rospy.loginfo("RobotBase::Subscribed to " +
            self.user_input_sub.resolved_name)


    def publish_web_page(self, web_name):
        msg = WebGoTo()
        msg.type = WebGoTo.TOUCH_PAGE
        msg.value = web_name
        self.web_publisher.publish(msg)


    def say(self, msg):
        goal = TtsGoal()
        goal.rawtext.text = msg
        goal.rawtext.lang_id = "en_GB"
        self.tts.send_goal_and_wait(goal)

    def user_input_touch_cb(self, data):
        """ Callback called when the user presses a button on the
        touchscreen.
        """
        if data.action:
            data = str(data.action).lower()
            rospy.loginfo("Button pressed: " + data)

            if data == "yes":
                self.say("Wonderful")
            elif data == "no":
                self.say("No problem! Maybe later")

            self.publish_web_page('new')



if __name__ == '__main__':
    print("Starting robot demo...")
    try:
        rospy.init_node('first_interaction_demo')
        robot = RobotBase()
        robot.publish_web_page('new')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
