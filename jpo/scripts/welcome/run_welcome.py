#!/usr/bin/env python
import rospy
from pal_detection_msgs.msg import Detections2d
from std_msgs.msg import String

def callback(data):
    if (len(data.detections) > 0):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.detections[0])
        coucou()


def coucou():
    rospy.loginfo('Hello')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('run_welcome', anonymous=True)
    rospy.loginfo('Created node run_welcome')

    rospy.Subscriber('/person_detector/detections', Detections2d, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
