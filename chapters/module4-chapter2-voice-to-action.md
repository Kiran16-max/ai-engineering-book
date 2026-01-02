---
id: module4-chapter2-voice-to-action
title: 'Module 4 – Chapter 2: Voice-to-Action with OpenAI Whisper'
sidebar_label: 'Module 4 – Chapter 2: Voice-to-Action'
---

## Chapter Overview

In this chapter, we will explore how to create a voice-to-action system for our humanoid robot using OpenAI's Whisper, a state-of-the-art automatic speech recognition (ASR) model. This will allow us to give commands to our robot using natural language, making the interaction more intuitive and human-like.

### Learning Goals

- Understand the fundamentals of automatic speech recognition.
- Learn how to use the OpenAI Whisper API to transcribe speech to text.
- Integrate Whisper with ROS 2 to create a voice-controlled robotic system.

## Automatic Speech Recognition (ASR)

ASR is a technology that allows a computer to convert spoken language into written text. Modern ASR systems like Whisper use deep learning to achieve high accuracy, even in noisy environments.

## Using the OpenAI Whisper API

The Whisper API is a simple and powerful tool for transcribing audio files. Here's a basic example of how to use it in Python:

```python
import openai

# Make sure you have your OpenAI API key set as an environment variable
# export OPENAI_API_KEY='your-api-key'

audio_file = open("path/to/your/audio.mp3", "rb")
transcription = openai.Audio.transcribe("whisper-1", audio_file)

print(transcription['text'])
```

### Real-time Speech Recognition

For a voice-controlled robot, we need to be able to transcribe speech in real-time. This can be achieved by capturing audio from a microphone in chunks, and sending each chunk to the Whisper API for transcription.

Here is a conceptual example of a ROS 2 node that captures audio and publishes the transcribed text:

```python
# conceptual_whisper_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, 'transcribed_text', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def listen_and_publish(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("Listening for commands...")
            while rclpy.ok():
                audio = self.recognizer.listen(source)
                try:
                    # This is a conceptual example. The actual implementation would
                    # involve calling the Whisper API.
                    text = self.recognizer.recognize_whisper_api(audio)
                    self.get_logger().info(f"Transcribed: {text}")
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
                except sr.UnknownValueError:
                    self.get_logger().info("Could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().error(f"Could not request results from Whisper API; {e}")

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    whisper_node.listen_and_publish()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1.  **Basic Transcription**: Write a Python script that takes an audio file as input and prints the transcribed text using the Whisper API.
2.  **ROS 2 Integration**: Create a ROS 2 package that contains a node that subscribes to an audio topic, transcribes the audio using Whisper, and publishes the transcribed text to another topic.
3.  **Command Recognition**: Extend the ROS 2 node from the previous exercise to recognize specific commands (e.g., "move forward," "stop"). When a command is recognized, the node should publish a corresponding message to a `cmd_vel` topic.
