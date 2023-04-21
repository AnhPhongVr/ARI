#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from std_msgs.msg import String
from pocketsphinx import LiveSpeech, get_model_path
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient


def text_to_speech(text):
    # Connexion au serveur d'action TTS
    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()

    # Création de l'objectif TTS pour dire notre phrase
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    client.send_goal_and_wait(goal)


def speech_recognition():
    # Initialisation du noeud ROS
    rospy.init_node('speech_recognition', anonymous=True)

    # Configuration de Pocketsphinx pour la reconnaissance vocale
    model_path = get_model_path()
    
    speech = LiveSpeech(
    verbose=False,
    sampling_rate=16000,
    buffer_size=2048,
    no_search=False,
    full_utt=False,
    hmm=os.path.join(model_path, 'en-us'),
    lm=os.path.join(model_path, 'en-us.lm.bin'),
    dic=os.path.join(model_path, 'cmudict-en-us.dict'),
)


    text_to_speech("talk !")
    # Boucle pour écouter en continu
    for phrase in speech:
        # On affiche la phrase entendue
        rospy.loginfo("I heard: " + str(phrase))
        # On vérifie que la phrase contient "hello" pour répondre
        if 'hello' in str(phrase):
            response = "Hello, how are you?"
            rospy.loginfo(response)
            text_to_speech(response)
            rospy.loginfo("Robot: " + response)
        elif str(phrase) != '' :
            text_to_speech("I heard " + str(phrase))

        else :
            rospy.loginfo("nothing")
    speech.stop()
    text_to_speech(" ")
    speech.start()

if __name__ == '__main__':
    try:
        speech_recognition()
    except rospy.ROSInterruptException:
        pass

