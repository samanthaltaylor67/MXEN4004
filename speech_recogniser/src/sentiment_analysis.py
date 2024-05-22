#!/usr/bin/python3

# Author: Samantha Taylor
# Purpose: Initialises sentiment_analysis node to command illumination LEDs according to sentiment (happy, angry, sad or other).
# Institution: Curtin University
# Unit: MXEN4004 - Mechatronic Engineering Research Project II
# Date: Semester 2, 2024

import rospy
from std_msgs.msg import UInt32MultiArray, String, Bool
import os

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, GdkPixbuf, Gdk, GLib

class sentiment_analysis:
    def __init__(self):
        # Initialise sentiment_analysis node
        rospy.init_node('sentiment_analysis', anonymous=True)

        # Get robot name
        self.robot_name = os.getenv("MIRO_ROBOT_NAME")
        topic_base_name = "/" + self.robot_name

        # Publisher
        self.pub_illum = rospy.Publisher(topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0)

        # Subscribers
        rospy.Subscriber("/speech_recogniser/sentiment_analysis", String, self.callback_sentiment)
        rospy.Subscriber('/speech_recogniser/end_conversation', Bool, self.callback_killall)

        # Initialise illumination data (6 lights - 3 per side of body)
        self.front_left, self.mid_left, self.rear_left, self.front_right, self.mid_right, self.rear_right = range(6)
        self.illum = UInt32MultiArray()
        self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

    def callback_killall(self, data):
        # Kill sentiment_analysis node
        if data.data:
            light_off = UInt32MultiArray()
            light_off.data = [0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000]
            self.pub_illum.publish(light_off)
            rospy.signal_shutdown("Conversation Ended")

    def callback_sentiment(self, data):
        sentiment = data.data

        # Set illumination data
        if sentiment.lower() == "happy":  # Green
            self.on_LEDChange(Gdk.RGBA(0.0, 255.0, 0.0, 1.0), 100)
        elif sentiment.lower() == "angry":  # Red
            self.on_LEDChange(Gdk.RGBA(255.0, 0.0, 0.0, 1.0), 100)
        elif sentiment.lower() == "sad":  # Blue
            self.on_LEDChange(Gdk.RGBA(0.0, 0.0, 255.0, 1.0), 100)
        elif sentiment.lower() == "other":  # Yellow
            self.on_LEDChange(Gdk.RGBA(255.0, 255.0, 0.0, 1.0), 100)
        else:
            self.on_LEDChange(Gdk.RGBA(255.0, 255.0, 255.0, 1.0), 100)

        self.pub_illum.publish(self.illum)

    def generate_illum(self, colour, bright):
        rgb_str = colour.to_string()
        useless, rgb_values = rgb_str.split("(")
        rgb_values, useless = rgb_values.split(")")
        r, g, b = rgb_values.split(",")
        r = int(r)
        g = int(g)
        b = int(b)
        return (int(bright) << 24) | (r << 16) | (g << 8) | b

    def on_LEDChange(self, rgb, bright):
        value = self.generate_illum(rgb, bright)

        self.illum.data[self.front_left] = value
        self.illum.data[self.front_right] = value
        self.illum.data[self.mid_left] = value
        self.illum.data[self.mid_right] = value
        self.illum.data[self.rear_left] = value
        self.illum.data[self.rear_right] = value

if __name__ == "__main__":
    sentiment_analysis = sentiment_analysis()
    rospy.spin()
