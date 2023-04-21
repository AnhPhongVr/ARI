#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from std_msgs.msg import String
from pocketsphinx import LiveSpeech, get_model_path
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient
import threading

class SpeechRecognition:
    def __init__(self):
        # Configuration de Pocketsphinx pour la reconnaissance vocale
        model_path = get_model_path()
        self.speech = LiveSpeech(
            verbose=False,
            sampling_rate=16000,
            buffer_size=2048,
            no_search=False,
            full_utt=False,
            hmm=os.path.join(model_path, 'en-us'),
            lm=os.path.join(model_path, 'en-us.lm.bin'),
            dic=os.path.join(model_path, 'cmudict-en-us.dict')
        )
    
        self.remaining_time = 20

        self.last_phrase = []
        self.stop = False
        self.all_phrases = ""

    def count(self):
        self.text_to_speech("talk")
        while self.remaining_time > 0:
            rospy.loginfo("Remaining time: " + str(self.remaining_time) + " seconds")
            rospy.sleep(1)
            self.remaining_time -= 1
        self.stop = True
        rospy.signal_shutdown("Time is up")
        
    def start_listening(self):
        
        # Boucle pour écouter en continu
        for phrase in self.speech:

            # On affiche la phrase entendue
            rospy.loginfo("I heard: " + str(phrase))
            self.last_phrase.append(str(phrase))
            # On vérifie que la phrase contient "hello" pour répondre
            if 'hello' in str(phrase):
                response = "Hello, how are you?"
                rospy.loginfo(response)
                self.text_to_speech(response)
                rospy.loginfo("Robot: " + response)
            if self.stop:
                self.text_to_speech(str(phrase))
                self.all_phrases = " ".join(self.last_phrase)
                rospy.sleep(1)
                self.text_to_speech("You said :")
                self.text_to_speech(self.all_phrases)
                rospy.loginfo("Robot: " + self.all_phrases)
                break
        
        rospy.loginfo(self.stop)    
        self.text_to_speech("finished")
        self.all_phrases = " ".join(self.last_phrase)
        rospy.sleep(1)
        self.text_to_speech("You said :")
        self.text_to_speech(self.all_phrases)
        rospy.loginfo("Robot: " + self.all_phrases)
                 
    def text_to_speech(self, text):
        # Connexion au serveur d'action TTS
        client = SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()

        # Création de l'objectif TTS pour dire notre phrase
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)

class CountDown:
    def __init__(self, duration):
        self.duration = duration
        self.remaining_time = duration
        
    def start(self):
        while self.remaining_time > 0:
            rospy.loginfo("Remaining time: " + str(self.remaining_time) + " seconds")
            rospy.sleep(1)
            self.remaining_time -= 1
        rospy.signal_shutdown("Time is up")
        
        

if __name__ == '__main__':
    try:
        rospy.init_node('speech_recognition', anonymous=True)
        speech_recognition = SpeechRecognition()
        
        
        # Lancement des threads
        thread1 = threading.Thread(target=speech_recognition.start_listening)
        thread2 = threading.Thread(target=speech_recognition.count)
        
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
    except rospy.ROSInterruptException:
        pass

