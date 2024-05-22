#!/usr/bin/python3

# Author: Samantha Taylor
# Purpose: Initialises recogniser node to listen through the MiRo microphone for speech.
# Institution: Curtin University
# Unit: MXEN4004 - Mechatronic Engineering Research Project II
# Date: Semester 2, 2024

import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray, String, Bool
import std_msgs

import time
import sys
import copy
import os
import numpy as np
import wave, struct
import webrtcvad
import pandas as pd
from scipy.io import wavfile
import openai
from dotenv import load_dotenv

import miro2 as miro

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 2

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

def error(msg):
    print(msg)
    sys.exit(0)

class MicrophoneFrame:
    def __init__(self):
        self.data = None

    def left(self):
        if self.data is not None:
            return self.data[:, 0]
        else:
            return None

    def right(self):
        if self.data is not None:
            return self.data[:, 1]
        else:
            return None

    def tail(self):
        if self.data is not None:
            return self.data[:, 2]
        else:
            return None

    def head(self):
        if self.data is not None:
            return self.data[:, 3]
        else:
            return None

class RobotInterfaceCallbacks:

    def __init__(self):
        self.cameras = None
        self.camera_left = None
        self.camera_right = None
        self.sensors = None
        self.microphones = None

class RobotInterfaceOutputValue:

    def __init__(self, default_value):

        self.default_value = default_value
        self.value = copy.deepcopy(self.default_value)
        self.timeout = 0

    def set(self, value, timeout):

        self.value = value
        self.timeout = int(timeout * 50.0)

    def get(self):

        ret = copy.deepcopy(self.value)
        if self.timeout > 0:
            self.timeout -= 1
            if self.timeout == 0:
                self.value = copy.deepcopy(self.default_value)
        return ret

class client:
    def __init__(self, node_name="recogniser", robot_name=None,
                 control_period=0.02, flags=0, command=None, mirocode_mode=False,
                 use_pose_control=False, wait_for_ready=True):
        # Initialisation #

        # Debug
        self.n_debug = -1
        self.t_debug = time.time()
        if os.path.isfile("/tmp/RobotInterface.debug"):
            self.n_debug = 0

        # Arguments
        self.node_name = node_name
        self.robot_name = robot_name
        self.control_period = control_period
        self.mirocode_mode = mirocode_mode
        self.use_pose_control = use_pose_control

        # Callbacks
        self.callbacks = RobotInterfaceCallbacks()

        # List of generated warnings
        self.warned = []

        # List of accepted flag commands
        self.flag_cmds = [None, "set", "clear", "toggle"]

        # Get default robot name from environment
        if self.robot_name is None:
            self.robot_name = os.getenv("MIRO_ROBOT_NAME")

        # Check that robot name has been correctly set
        assert self.robot_name is not None, "MIRO_ROBOT_NAME environment variable has\
    					not been set. Please set it or specify robot_name"

        # Interface state
        self.active = True

        # Microphones
        self.mics = MicrophoneFrame()

        # Flags state
        default = std_msgs.msg.UInt32()
        # We disable status LEDs as standard and add any flags from init
        default.data = miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS | flags
        self.msg_flags = RobotInterfaceOutputValue(default)

        # Initialise the  ROS node
        if self.node_name is not None:
            rospy.init_node(self.node_name, anonymous=True)

        # Set initial state
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.micbuf_temp = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.buffer_stuff = 0
        self.mirocode_mode = mirocode_mode

        # Robot ROS topics #
        topic_base_name = "/" + self.robot_name

        # Publishers
        topic = topic_base_name + "/control/flags"
        self.pub_flags = rospy.Publisher(topic, std_msgs.msg.UInt32, queue_size=0)

        # Subscribers
        topic = topic_base_name + "/sensors/mics"
        self.mics_sub = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics, tcp_nodelay=True)

        sys.stdout.flush()

        # Send flags
        self.pub_flags.publish(self.msg_flags.get())

        # Release threads
        self.release_threads = True

        # Voice Activity Detection #
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3) # 3 Agressiveness

        # Wait for connect
        time.sleep(0.5)

        # Transcription #
        self.whisper_model = "whisper-1"
        load_dotenv()
        openai.api_key = os.getenv("OPENAI_API_KEY")

        # Publishers
        self.pub = rospy.Publisher('/speech_recogniser/transcription', String, queue_size=10)
        self.rate = rospy.Rate(1)

        # Subscribers
        rospy.Subscriber('/speech_recogniser/conversing', Bool, self.callback_converse)
        rospy.Subscriber('/speech_recogniser/end_conversation', Bool, self.callback_killall)

    def callback_killall(self, data):
        # Kill recogniser node
        if data.data:
            rospy.signal_shutdown("Conversation Ended")

    def callback_converse(self, data):
        # Restart recording
        if not data.data:
            self.micbuf = np.zeros((0, 4), 'uint16')
            self.outbuf = None

    def voice_detection(self, filename):
        # Return if speech is detected within the one second recording
        sample_rate, samples = wavfile.read(filename)

        raw_samples = struct.pack("%dh" % len(samples), *samples)
        window_duration = 0.03  # duration in seconds
        samples_per_window = int(window_duration * sample_rate + 0.5)
        bytes_per_sample = 2

        segments = []

        overall = False

        for start in np.arange(0, len(samples), samples_per_window):
            stop = min(start + samples_per_window, len(samples))

            is_speech = self.vad.is_speech(raw_samples[start * bytes_per_sample: stop * bytes_per_sample],
                                      sample_rate=sample_rate)

            if is_speech:
                overall = True

            segments.append(dict(
                start=start,
                stop=stop,
                is_speech=is_speech))

        return overall

    def callback_mics(self, msg):
        # Sample rate = 20,000Hz, therefore records 20,000 samples per second. Callback is called a maximum of
        # 40 times - (msg.data size = 500), 20,000/500 = 40 times. Sometimes will not fill the entire buffer.
        # When the buffer is full it will restart

        # If recording
        if not self.micbuf is None:
            self.micbuf_temp = np.concatenate((self.micbuf_temp, np.transpose(np.asarray(msg.data).reshape((4, 500)))))

            # Report recording progress to terminal
            sys.stdout.write(".")
            sys.stdout.flush()

            # Check every second for speech
            if self.micbuf_temp.shape[0] >= MIC_SAMPLE_RATE:
                data = np.array(self.micbuf_temp)
                result = data[:, 1]

                # Output every second of speech to file to be analysed for speech.
                outfilename = os.getcwd() + '/client_audio.wav'
                file = wave.open(outfilename, 'wb')
                file.setsampwidth(2)
                # vad only accepts sample rate of 8000, 16 000, 32 000 and 48 000Hz. Downsample from 20 000Hz to 16 000Hz.
                file.setframerate(16000)

                file.setnchannels(1)
                x = np.reshape(result, (-1))
                for s in x:
                    file.writeframes(struct.pack('<h', s))

                is_speech = self.voice_detection(outfilename)
                os.remove(outfilename)

                # Speech is detected - still speaking
                if is_speech:
                    self.micbuf = np.concatenate((self.micbuf, self.micbuf_temp))
                else:
                    # End the recording - end of speech
                    self.outbuf = self.micbuf

                    outfilename = os.getcwd() + '/outbuf.wav'
                    file = wave.open(outfilename, 'wb')
                    file.setsampwidth(2)
                    file.setframerate(MIC_SAMPLE_RATE)

                    file.setnchannels(1)
                    x = np.reshape(self.outbuf, (-1))
                    for s in x:
                        file.writeframes(struct.pack('<h', s))

                    print(" OK!")
                    self.micbuf = None

                # Reset the buffer
                self.micbuf_temp = np.zeros((0, 4), 'uint16')

    def disconnect(self):
        # Disconnect callbacks
        self.callbacks = RobotInterfaceCallbacks()

        self.term()

        # Wait for threads to terminate
        if not self.mirocode_mode:
            sys.stdout.write("wait for robot thread terminate...\n")
        if not self.mirocode_mode:
            print("...OK")

        # Disconnect from robot (just wait)
        sys.stdout.write("disconnecting from robot...\n")
        sys.stdout.flush()
        self.sub_package = None
        time.sleep(0.5)
        print("...OK")

    def verbose_print(self, *args):
        if not self.mirocode_mode:
            print(" ".join(args))

    def loop(self):
        # Loop
        while not rospy.core.is_shutdown():
            # If recording finished
            if not self.outbuf is None:
                break

            # State
            time.sleep(0.02)

            outfilename = os.getcwd() + '/user_speech.wav'
            file = wave.open(outfilename, 'wb')
            file.setsampwidth(2)
            file.setframerate(MIC_SAMPLE_RATE)

            # 1 channel = right ear
            file.setnchannels(1)
            if self.outbuf is not None:
                x = np.reshape(self.outbuf[:,1], (-1))
                for s in x:
                    file.writeframes(struct.pack('<h', s))

                # Close file
                file.close()

                speech = open(outfilename, 'rb')
                start = time.time()

                # Transcribe
                try:
                    wcompletion = openai.Audio.transcribe(
                        model=self.whisper_model,
                        file=speech
                    )
                    end = time.time()

                    # Returns the transcription of the audio file
                    user_input = wcompletion['text']
                    print("[User]: " + str(user_input))
                    #print("Transcribed in {0:.5f}s".format(end - start))

                    self.pub.publish(user_input)
                except:
                    # No speech detected within the first second, therefore restart the microphone
                    self.micbuf = np.zeros((0, 4), 'uint16')

                # Reset for next recording
                self.rate.sleep()
                os.remove(outfilename)
                self.outbuf = None

if __name__ == "__main__":
    main = client()
    main.loop()
    rospy.spin()
