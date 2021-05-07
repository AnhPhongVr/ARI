#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

def callback(data):
    listButtons = detectPushedButton(data)
    mkStringMessage = ', '.join(listButtons)
    
    rospy.loginfo("You pressed buttons " + mkStringMessage)

def detectPushedButton(data):
    listButtons = []

    if (data.buttons[0] != 0): listButtons.append("X")
    if (data.buttons[1] != 0): listButtons.append("A")
    if (data.buttons[2] != 0): listButtons.append("B")
    if (data.buttons[3] != 0): listButtons.append("Y")
    if (data.buttons[4] != 0): listButtons.append("LB")
    if (data.buttons[5] != 0): listButtons.append("RB")
    if (data.buttons[6] != 0): listButtons.append("LT")
    if (data.buttons[7] != 0): listButtons.append("RT")
    if (data.buttons[8] != 0): listButtons.append("back")
    if (data.buttons[9] != 0): listButtons.append("start")
    if (data.buttons[10] != 0): listButtons.append("Button stick left")  
    if (data.buttons[11] != 0): listButtons.append("Button stick right")

    if (data.axes[0] != 0): listButtons.append("Left/Right Axis stick left")
    if (data.axes[1] != 0): listButtons.append("Up/Down Axis stick left")
    if (data.axes[2] != 0): listButtons.append("Left/Right Axis stick right")
    if (data.axes[3] != 0): listButtons.append("Up/Down Axis stick right")
    if (data.axes[4] != 0): listButtons.append("cross key left/right")
    if (data.axes[5] != 0): listButtons.append("cross key up/down")

    return listButtons 

def main():
    rospy.init_node('run_joy_ari', anonymous=True)
    rospy.loginfo('Created node run_joy_ari')

    rospy.Subscriber('/joy', Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
