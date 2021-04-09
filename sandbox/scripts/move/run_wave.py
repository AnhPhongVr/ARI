#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def wave():
    pub = rospy.Publisher('/arm_left_controller/command', JointTrajectory, queue_size=10)
    pub.
    rospy.init_node('run_wave', anonymous=True)
    state_is_finish = False

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown() and not state_is_finish:
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        points_first_move = JointTrajectoryPoint()
        points_first_move.positions = [0, 0, 0, 0]
        points_first_move.velocities = [0.1, 0.1, 0.1, 0.1]
        points_first_move.time_from_start = rospy.Duration(3)
        #rospy.loginfo(points_first_move.__getstate__)
        
        points_second_move = JointTrajectoryPoint()
        points_second_move.positions = [-0.24, 1.79, 0.49, 0]
        points_second_move.velocities = [0.1, 0.1, 0.1, 0.1]
        points_second_move.time_from_start = rospy.Duration(6)

        points_third_move = JointTrajectoryPoint()
        points_third_move.positions = [0, 0, 0, 0]
        points_third_move.velocities = [0.1, 0.1, 0.1, 0.1]
        points_third_move.time_from_start = rospy.Duration(9)

        first_move = JointTrajectory()
        #first_move.header = 
        first_move.joint_names = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint"]
        first_move.points = [points_first_move, points_second_move, points_third_move]
        
        pub.publish(first_move)
        rate.sleep()

        #state_is_finish = True

if __name__ == '__main__':
    try:
        wave()
    except rospy.ROSInterruptException:
        pass

"""
/arm_left-cotroller/follow_joint_trajectory/feedback -> position désiré et actuel (se met a jour qund on envoie une action)
/arm_left-cotroller/follow_joint_trajectory/result -> se met a jour une fois que l'action demandé a été éffectué 

"""
