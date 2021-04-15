#!/usr/bin/env python

import rospy

from actionlib import SimpleActionClient

from sensor_msgs.msg import Joy

def callback(data):
    rospy.loginfo(data.axes[0])
    rospy.loginfo(data.buttons[0])

def main():
    rospy.init_node('run_joy_ari', anonymous=True)
    rospy.loginfo('Created node run_joy_ari')

    rospy.Subscriber('/joy', Joy, callback) #joy
    rospy.spin()

if __name__ == '__main__':
    main()
