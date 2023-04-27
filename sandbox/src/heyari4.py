#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient
# import openai
import re
# from kaldi_decoder import KaldiDecoder # Importer la classe KaldiDecoder
from kaldiasr.nnet3 import KaldiNNet3OnlineModel, KaldiNNet3OnlineDecoder
# Initialisation du noeud ROS
rospy.init_node('voice_control', anonymous=True)

# Connexion au serveur d'action TTS
client = SimpleActionClient('/tts', TtsAction)
client.wait_for_server()

MODELDIR = 'data/models/kaldi-generic-en-tdnn_sp-latest'
kaldi_model = KaldiNNet3OnlineModel (MODELDIR)
decoder = KaldiNNet3OnlineDecoder (kaldi_model)

# Configuration de l'API OpenAI
# openai.api_key = "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" # Mettre votre clé API ici

def text_to_speech(text):
    # Création de l'objectif TTS pour dire notre phrase
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    client.send_goal_and_wait(goal)


def generate_completion(prompt):
    # Appel de l'API OpenAI pour générer une complétion de texte
    completions = openai.Completion.create(
        engine="davinci",
        prompt=prompt,
        max_tokens=60,
        n=1,
        stop=None,
        temperature=0.7,
    )

    # Récupération de la première suggestion de complétion
    message = completions.choices[0].text

    # Nettoyage du texte en retirant les caractères spéciaux et les sauts de ligne
    message = re.sub('[^0-9a-zA-Z\n\.]', ' ', message)
    message = message.replace('\n', ' ')

    return message

def speech_to_text(audio_file):
    # Initialisation du décodeur Kaldi
    decoder = KaldiNNet3OnlineDecoder (kaldi_model)

    # Transcription de la parole en texte à l'aide de PyKaldi
    text = decoder.decode_wav(audio_file)

    return text

def voice_control():
    global client

    # Configuration du module Kaldi
    r = sr.Recognizer()
    r.energy_threshold = 4000

    # Message à dire pour indiquer que le robot écoute
    text_to_speech("Hey, how can I help you?")

    # Boucle pour écouter en continu
    while not rospy.is_shutdown():
        # Enregistrement audio de l'utilisateur
        audio_file = record_audio()

        # Transcription de la parole en texte avec Kaldi
        with sr.AudioFile(audio_file) as source:
            audio = r.record(source)
        try:
            text = r.recognize_kaldi(audio)
        except sr.UnknownValueError:
            text = ""
        except sr.RequestError as e:
            rospy.logerr("Kaldi error; {0}".format(e))
            text = ""

        # On affiche la phrase entendue
        rospy.loginfo("I heard: " + text)

        if "ari" in text.lower():
            # Message à dire pour indiquer que le robot a compris la commande
            text_to_speech("Yes, I'm listening")

            # Boucle pour écouter la suite de la commande
            while not rospy.is_shutdown():
                # Enregistrement audio de l'utilisateur
                audio_file = record_audio()

                # Transcription de la parole en texte avec Kaldi
                with sr.AudioFile(audio_file) as source:
                    audio = r.record(source)
                try:
                    text = r.recognize_kaldi(audio)
                except sr.UnknownValueError:
                    text = ""
                except sr.RequestError as e:
                    rospy.logerr("Kaldi error; {0}".format(e))
                    text = ""

                rospy.loginfo("I heard: " + text)

                if "stop" in text.lower():
                    # Message à dire pour indiquer que le robot arrête d'écouter
                    text_to_speech("Okay, let me know if you need anything")
                    break
                else:
                    # Message à dire pour confirmer la commande
                    text_to_speech("Did you say " + text + "?")
                    rospy.sleep(1) # Pause d'une seconde pour laisser le temps à l'utilisateur de répondre
                    for i in range(3):
                        # Enregistrement audio de la réponse de l'utilisateur
                        audio_file = record_audio()

                        # Transcription de la parole en texte avec Kaldi
                        with sr.AudioFile(audio_file) as source:
                            audio = r.record(source)
                        try:
                            text = r.recognize_kaldi(audio)
                        except sr.UnknownValueError:
                            text = ""
                        except sr.RequestError as e:
                            rospy.logerr("Kaldi error; {0}".format(e))
                            text = ""

                        if "yes" in text.lower():
                            # Message à dire pour indiquer que le robot exécute la commande
                            text_to_speech("Okay, I will do that")

                            # On génère la complétion avec l'API de ChatGPT
                            prompt = text # On récupère la commande dans une variable
                            # On génère la complétion avec l'API de ChatGPT
                            # api_key = "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" # Remplacer par votre clé API
                            # headers = {
                            #    "Content-Type": "application/json",
                            #    "Authorization": "Bearer " + api_key
                            #}
                            #data = {
                            #    "prompt": prompt,
                            #    "temperature": 0.7,
                            #    "max_tokens": 60,
                            #    "stop": ["\n"]
                            #}
                            #response = requests.post("https://api.openai.com/v1/engines/davinci-codex/completions", json=data, headers=headers)
                            # On récupère la réponse de l'API et on la met en forme
                            #if response.ok:
                            #    text = response.json()["choices"][0]["text"].strip()
                            #    text = re.sub(r'\s+', ' ', text) # On enlève les espaces inutiles
                                # On utilise la fonction text_to_speech pour dire la réponse
                            #    text_to_speech(text)
                            #else:
                            #    rospy.logerr("An error occurred while calling the ChatGPT API")
                            break
                        elif "no" in text.lower():
                            # Message à dire pour indiquer que le robot demande la commande à nouveau
                            text_to_speech("I'm sorry, can you repeat that?")
                            break
                        else:
                            text_to_speech("I'm sorry, I didn't understand. Please say yes or no.")

if __name__ == '__main__':
    try:
        voice_control()
    except rospy.ROSInterruptException:
        pass
