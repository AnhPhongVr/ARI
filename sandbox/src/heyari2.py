#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from std_msgs.msg import String
from pocketsphinx import LiveSpeech, get_model_path
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient
import openai
import re

# Initialisation du noeud ROS
rospy.init_node('voice_control', anonymous=True)

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
    dic=os.path.join(model_path, 'cmudict-en-us.dict')
)

# Connexion au serveur d'action TTS
client = SimpleActionClient('/tts', TtsAction)
client.wait_for_server()

# Configuration de l'API OpenAI
openai.api_key = "YOUR_API_KEY" # Mettre votre clé API ici

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

def voice_control():
    global speech
    global client

    # Message à dire pour indiquer que le robot écoute
    text_to_speech("Hey, how can I help you?")

    # Boucle pour écouter en continu
    for phrase in speech:
        # On affiche la phrase entendue
        rospy.loginfo("I heard: " + str(phrase))
        if "ari" in str(phrase).lower():
            # Message à dire pour indiquer que le robot a compris la commande
            text_to_speech("Yes, I'm listening")

            # Boucle pour écouter la suite de la commande
            for phrase2 in speech:
                rospy.loginfo("I heard: " + str(phrase2))
                if "stop" in str(phrase2).lower():
                    # Message à dire pour indiquer que le robot arrête d'écouter
                    text_to_speech("Okay, let me know if you need anything")
                    break
                else:
                    # Message à dire pour confirmer la commande
                    text_to_speech("Did you say " + str(phrase2) + "?")
                    rospy.sleep(1) # Pause d'une seconde pour laisser le temps à l'utilisateur de répondre
                    for phrase3 in speech:
                        if "yes" in str(phrase3).lower():
                            # Message à dire pour indiquer que le robot exécute la commande
                            text_to_speech("Okay, I will do that")
                            prompt = str(phrase2) # On récupère la commande dans une variable
                            # On génère la complétion avec l'API de ChatGPT
                            api_key = "YOUR_API_KEY" # Remplacer par votre clé API
                            headers = {
                                "Content-Type": "application/json",
                                "Authorization": f"Bearer {api_key}"
                            }
                            data = {
                                "prompt": prompt,
                                "temperature": 0.7,
                                "max_tokens": 60,
                                "stop": ["\n"]
                            }
                            response = requests.post("https://api.openai.com/v1/engines/davinci-codex/completions", json=data, headers=headers)
                            # On récupère la réponse de l'API et on la met en forme
                            if response.ok:
                                text = response.json()["choices"][0]["text"].strip()
                                text = re.sub(r'\s+', ' ', text) # On enlève les espaces inutiles
                                # On utilise la fonction text_to_speech pour dire la réponse
                                text_to_speech(text)
                            else:
                                rospy.logerr("An error occurred while calling the ChatGPT API")
                            break
                        elif "no" in str(phrase3).lower():
                            # Message à dire pour indiquer que le robot demande la commande à nouveau
                            text_to_speech("I'm sorry, can you repeat that?")
                            break

if __name__ == '__main__':
    try:
        voice_control()
    except rospy.ROSInterruptException:
        pass
