#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pyaudio
import wave
import openai

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

def callback(data):
    # start Recording
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print("recording...")
    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    print("finished recording")

    # stop Recording
    stream.stop_stream()
    stream.close()
    p.terminate()

    # write audio to file
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    # transcribe audio using OpenAI API
    audio_file = open(WAVE_OUTPUT_FILENAME, 'rb')
    transcript = openai.Audio.transcribe("whisper-1", audio_file)
    print(transcript)

def listener():
    rospy.init_node('speech_recognition', anonymous=True)
    rospy.Subscriber('microphone', String, callback)
    rospy.spin()

if __name__ == '__main__':
    openai.api_key = "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"
    listener()

