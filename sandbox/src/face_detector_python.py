#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from actionlib import SimpleActionClient
# To get the type of msg we will need if we have the robot running:
#   rostopic type /tts/goal
#   pal_interaction_msgs/TtsActionGoal
# Action servers always have a type XXXXAction
# and the goals are always XXXXGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

class FaceDetector:
    def __init__(self):
        rospy.init_node('face_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_decompressed', Image, self.detect_faces_callback)
        self.image_pub = rospy.Publisher('/face_detector/image_with_rectangles', Image, queue_size=10)
        rospy.loginfo('Face detector node started.')
        self.text_to_speech("Please smile, you are on camera")
        self.is_smiling = True
    
        

    def text_to_speech(self,text):
        client = SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)


    def perform_motion(self):

        self.motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
        self.motion_client.wait_for_server()
        goal = PlayMotionGoal()
        goal.motion_name = "nod"
        goal.skip_planning = False
        self.motion_client.send_goal_and_wait(goal)
        rospy.loginfo("Movement done")

    def detect_faces_callback(self, data):
        
        rospy.loginfo("je suis dans la methode detect_faces_callback")
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("date value is:")
            


        except Exception as e:
            print(e)
            return

        face_cascade = cv2.CascadeClassifier('/home/pal/ari_public_ws/src/ari_isep/sandbox/src/haarcascade_frontalface_default.xml')
        smile_cascade = cv2.CascadeClassifier('/home/pal/ari_public_ws/src/ari_isep/sandbox/src/haarcascade_smile.xml')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        rospy.loginfo("Dans le rectangle")
        faces = face_cascade.detectMultiScale(gray, 1.4, 7)

        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            rospy.loginfo('Face detected at ({}, {}) with width={} and height={}'.format(x, y, w, h))
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = cv_image[y:y+h, x:x+w]
            
            smiles = smile_cascade.detectMultiScale(roi_gray, 1.8, 20)
            if len(smiles) > 0:
                print("smile detected")
                for (sx, sy, sw, sh) in smiles:
                    cv2.rectangle(roi_color, (sx, sy), ((sx + sw), (sy + sh)), (0, 0, 255), 2)
                    cv2.putText(roi_color, "smile", (sx, sy),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    if self.is_smiling:
                        self.text_to_speech("Nice smile !")
                        self.perform_motion()
                        self.is_smiling = False
            else:
                print("smile not detected")
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except Exception as e:
            print(e)

if __name__ == '__main__':
    try:
        detector = FaceDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

