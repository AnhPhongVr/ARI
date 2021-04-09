#!/usr/bin/env python

import sys
import rospy
import roslaunch
from pal_detection_msgs.msg import Detections2d
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

detection_number = 0

def callback(data):
    global detection_number 

    if (detection_number == 0 and len(data.detections) > 0 ):
        rospy.loginfo("A new person is here!")
        try:
            wave()
            say_hello()
        except rospy.ROSInterruptException:
            pass

    detection_number = len(data.detections)  

def wave():
    rospy.loginfo('Ari is waving his hand')

    pub = rospy.Publisher('/arm_left_controller/command', JointTrajectory, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    if not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        points_first_move = JointTrajectoryPoint()
        points_first_move.positions = [0, 0, 0, 0]
        points_first_move.velocities = [0.1, 0.1, 0.1, 0.1]
        points_first_move.time_from_start = rospy.Duration(3)

        points_second_move = JointTrajectoryPoint()
        points_second_move.positions = [-0.24, 1.79, 0.49, 0]
        points_second_move.velocities = [0.1, 0.1, 0.1, 0.1]
        points_second_move.time_from_start = rospy.Duration(6)

        first_move = JointTrajectory()
        #first_move.header = 
        first_move.joint_names = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint"]
        first_move.points = [points_first_move, points_second_move]

        pub.publish(first_move)
        #rospy.Duration(3)
        rate.sleep()

def say_hello():
    rospy.loginfo('Ari is saying hello')

def main():
    rospy.init_node('run_welcome', anonymous=True)
    rospy.loginfo('Created node run_welcome')

    """
    string_path = sys.argv[0]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cli_args = ['/pal_person_detector_opencv/launch/detector.launch','image:=/head_front_camera/image_raw']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    rospy.loginfo("Started launch file")
    """

    rospy.Subscriber('/person_detector/detections', Detections2d, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

'''
Position 1
1.50
1.50
0.00
2.29

Position 2
1.50
1.50
0.00
1.40

roslaunch pal_person_detector_opencv detector.launch image:=/head_front_camera/image_raw
'''
