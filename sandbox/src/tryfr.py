#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient
import os

def text_to_speech(text, lang_id):
    # Connexion au serveur d'action TTS
    client = SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()

    # Création de l'objectif TTS pour dire notre phrase
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = lang_id
    client.send_goal_and_wait(goal)

def main():
    rospy.init_node('tts_node', anonymous=True)
    text_to_speech("Bonjour, comment allez-vous ?", "fr-FR")

if __name__ == '__main__':
    main()

