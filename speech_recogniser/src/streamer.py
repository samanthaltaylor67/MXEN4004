#!/usr/bin/env python3

import time
import sys
import os
import subprocess
import numpy as np
import wave
import pyttsx3
from std_msgs.msg import UInt16MultiArray, Int16MultiArray, String, Bool
import rospy
import contextlib

# Amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# Messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# Using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# Microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

TRACK_FILE = "tts_response.wav"

TRACK_PATH = os.path.join(os.getcwd(), TRACK_FILE)

################################################################

def error(msg):
    print(msg)
    sys.exit(0)

################################################################

# if the file is not there, fail
if not os.path.isfile(TRACK_PATH):
    print('file not found', TRACK_PATH)

class streamer:
    def callback_log(self, msg):
        sys.stdout.write(msg.data)
        sys.stdout.flush()

    def callback_stream(self, msg):
        self.buffer_space = msg.data[0]
        self.buffer_total = msg.data[1]

    def callback_response(self, data):
        response = data.data

        # Write string response to speech - TTS
        responsefile = os.path.join(os.getcwd(), TRACK_FILE)
        self.engine.save_to_file(response, responsefile)
        self.engine.runAndWait()

        file = TRACK_PATH + ".decode"
        # Decode wav
        if os.path.isfile(file):
            os.remove(file)

        cmd = "ffmpeg -y -i \"" + TRACK_PATH + "\" -f s16le -acodec pcm_s16le -ar 8000 -ac 1 \"" + file + "\""
        subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if not os.path.isfile(file):
            error('failed decode mp3')

        # Load wav
        with open(file, 'rb') as f:
            dat = f.read()
        data_r = 0

        # Convert to numpy array
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)

        # Normalise wav
        dat = dat.astype(np.float64)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()

        state_file = None

        # Periodic reports
        count = 0

        # Safety dropout if receiver not present
        dropout_data_r = -1
        dropout_count = 3

        # Loop
        while not rospy.core.is_shutdown():

            # Check state_file
            if not state_file is None:
                if not os.path.isfile(state_file):
                    break

            # If we've received a report
            if self.buffer_total > 0:

                # Compute amount to send
                buffer_rem = self.buffer_total - self.buffer_space
                n_bytes = BUFFER_STUFF_BYTES - buffer_rem
                n_bytes = max(n_bytes, 0)
                n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

                # If amount to send is non-zero
                if n_bytes > 0:
                    msg = Int16MultiArray(data=dat[data_r:data_r + n_bytes])
                    self.pub_stream.publish(msg)
                    data_r += n_bytes

            # Break
            if data_r >= len(dat):
                break

            # Report once per second
            if count == 0:
                count = 10

                # Check at those moments if we are making progress, also
                if dropout_data_r == data_r:
                    if dropout_count == 0:
                        print("dropping out because of no progress...")
                        break
                    print("dropping out in", str(dropout_count) + "...")
                    dropout_count -= 1
                else:
                    dropout_data_r = data_r

            # Count tenths
            count -= 1
            time.sleep(0.1)

        time.sleep(0.5)
        os.remove(responsefile + ".decode")

        # Restart the microphone
        self.pub_conversing.publish(False)

        # Note duration of TTS file
        with contextlib.closing(wave.open(TRACK_PATH, 'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            self.wavlength = frames / float(rate)
    def callback_killall(self, data):
        # Kill streamer node
        if data.data:
            time.sleep(self.wavlength + 1)
            rospy.signal_shutdown("Conversation Ended")

    def __init__(self):
        # Initialise the streamer node
        rospy.init_node('streamer', anonymous=True)

        # State
        self.buffer_space = 0
        self.buffer_total = 0
        self.wavlength = 0

        # Get robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Robot ROS topics #
        # Publishers
        topic = topic_base_name + "/control/stream"
        self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

        # Subscribers
        topic = topic_base_name + "/platform/log"
        self.sub_log = rospy.Subscriber(topic, String, self.callback_log, queue_size=5, tcp_nodelay=True)

        topic = topic_base_name + "/sensors/stream"
        self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1,
                                           tcp_nodelay=True)

        # Text to Speech #
        # Publishers
        self.pub_conversing = rospy.Publisher('/speech_recogniser/conversing', Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/speech_recogniser/response', String, self.callback_response)
        rospy.Subscriber('/speech_recogniser/end_conversation', Bool, self.callback_killall)

        # Initialisation of text to speech
        self.engine = pyttsx3.init()

if __name__ == "__main__":
    streamer = streamer()
    rospy.spin()