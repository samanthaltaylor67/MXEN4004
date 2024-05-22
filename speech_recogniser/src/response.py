#!/usr/bin/env python3

# Author: Samantha Taylor
# Purpose: Initialises response node to generate a response to the users input using OpenAI's ChatGPT-4.
# Institution: Curtin University
# Unit: MXEN4004 - Mechatronic Engineering Research Project II
# Date: Semester 2, 2024

import rospy
import openai
from std_msgs.msg import String, Bool
from dotenv import load_dotenv
import os
import time
import re

class response:
    def __init__(self):
        # Initialise the response node
        rospy.init_node('response', anonymous=True, disable_signals=True)

        # Response Generation #
        load_dotenv()
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.gpt_model = "gpt-4"

        # Stores the system context
        personalityFile = "/personality.txt"
        with open(os.getcwd() + personalityFile, "r") as file:
            mode = file.read()

        # Initialise system context and establish conversational history
        self.messages = [{"role": "system", "content": f"{mode}"}]

        # Start conversation flag
        self.converse = False

        # Command definition
        self.wakeword = r"hello[\s,]+GPT[!\.?]*"
        self.endword = r"goodbye[\s,]+GPT[!\.?]*"

        # Publishers
        self.pub_conversing = rospy.Publisher('/speech_recogniser/conversing', Bool, queue_size=10)
        self.pub_start = rospy.Publisher('/speech_recogniser/start', Bool, queue_size=10)
        self.pub_sentiment = rospy.Publisher('/speech_recogniser/sentiment_analysis', String, queue_size=10)
        self.pub_response = rospy.Publisher('/speech_recogniser/response', String, queue_size=10)
        self.pub_killall = rospy.Publisher('/speech_recogniser/end_conversation', Bool, queue_size=10)

        # Subscriber
        rospy.Subscriber('/speech_recogniser/transcription', String, self.callback_transcription)

    def callback_transcription(self, data):
        user_input = data.data

        wakeword_match = re.search(self.wakeword, user_input, re.IGNORECASE)

        if wakeword_match:
            self.pub_conversing.publish(True)
            self.pub_start.publish(True)
            self.converse = True


        if self.converse:
            self.messages.append({"role": "user", "content": user_input})

            start = time.time()

            completion = openai.ChatCompletion.create(
                model=self.gpt_model,
                messages=self.messages,
                temperature=0.8
            )
            end = time.time()

            init_response = completion.choices[0].message.content
            try:
                raw_response, raw_sentiment = init_response.split('---')

                # Trim any leading or trailing whitespaces from the response and sentiment
                response = raw_response.strip()
                sentiment = raw_sentiment.strip()
                self.pub_sentiment.publish(sentiment)
            except(ValueError):
                response = init_response
                self.pub_sentiment.publish("other")

            self.messages.append({"role": "assistant", "content": response})

            print("[MiRo]: " + str(response))
            #print("Response generated in {0:.5f}s".format(end - start))

            self.pub_response.publish(response)

            endword_match = re.search(self.endword, user_input, re.IGNORECASE)
            if endword_match:
                self.pub_killall.publish(True)
                rospy.signal_shutdown("Conversation Ended")
        else:
            self.pub_conversing.publish(False)

if __name__ == '__main__':
    response()
    rospy.spin()
