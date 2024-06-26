#!/usr/bin/env python3

"""
MiRo orienting towards a sound
"""

import os
import numpy as np
import rospy
import miro2 as miro
import geometry_msgs
from node_detect_audio_engine import DetectAudioEngine
from std_msgs.msg import Int16MultiArray
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist, TwistStamped
import time


try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2


class AudioClient():
   
    def __init__(self):       
        #Microphone Parameters
        # Number of points to display
        self.x_len = 40000
        # number of microphones coming through on topic
        self.no_of_mics = 4

        #Generate figure for plotting mics
        self.fig = plt.figure()
        self.fig.suptitle("Microphones") # Give figure title

        #HEAD
        self.head_plot = self.fig.add_subplot(4,1,3)
        self.head_plot.set_ylim([-33000, 33000])
        self.head_plot.set_xlim([0, self.x_len])
        self.head_xs = np.arange(0, self.x_len)
        self.head_plot.set_xticklabels([])
        self.head_plot.set_yticks([])
        self.head_plot.grid(which="both", axis="x")
        self.head_plot.set_ylabel("Head", rotation=0, ha="right")
        self.head_ys = np.zeros(self.x_len)
        self.head_line, = self.head_plot.plot(self.head_xs, self.head_ys, linewidth=0.5, color="g")


        #LEFT EAR
        self.left_ear_plot = self.fig.add_subplot(4,1,1)
        self.left_ear_plot.set_ylim([-33000, 33000])
        self.left_ear_plot.set_xlim([0, self.x_len])
        self.left_ear_xs = np.arange(0, self.x_len)
        self.left_ear_plot.set_xticklabels([])
        self.left_ear_plot.set_yticks([])
        self.left_ear_plot.grid(which="both", axis="x")
        self.left_ear_plot.set_ylabel("Left Ear", rotation=0, ha="right")
        self.left_ear_ys = np.zeros(self.x_len)
        self.left_ear_line, = self.left_ear_plot.plot(self.left_ear_xs, self.left_ear_ys, linewidth=0.5, color="b")

        #RIGHT EAR
        self.right_ear_plot = self.fig.add_subplot(4,1,2)
        self.right_ear_plot.set_ylim([-33000, 33000])
        self.right_ear_plot.set_xlim([0, self.x_len])
        self.right_ear_xs = np.arange(0, self.x_len)
        self.right_ear_plot.set_xticklabels([])
        self.right_ear_plot.set_yticks([])
        self.right_ear_plot.grid(which="both", axis="x")
        self.right_ear_plot.set_ylabel("Right Ear", rotation=0, ha="right")
        self.right_ear_ys = np.zeros(self.x_len)
        self.right_ear_line, = self.right_ear_plot.plot(self.right_ear_xs, self.right_ear_ys, linewidth=0.5, color="r")

        #Tail
        self.tail_plot = self.fig.add_subplot(4,1,4)
        self.tail_plot.set_ylim([-33000, 33000])
        self.tail_plot.set_xlim([0, self.x_len])
        self.tail_xs = np.arange(0, self.x_len)
        self.tail_plot.set_yticks([])
        self.tail_plot.set_xlabel("Samples")
        self.tail_plot.grid(which="both", axis="x")
        self.tail_plot.set_ylabel("Tail", rotation=0, ha="right")
        self.tail_ys = np.zeros(self.x_len)
        self.tail_line, = self.tail_plot.plot(self.tail_xs, self.tail_ys, linewidth=0.5, color="c")

        #self.ani = animation.FuncAnimation(self.fig, self.update_line, fargs=(self.left_ear_ys,self.right_ear_ys, self.head_ys, self.tail_ys,), init_func=self.animation_init, interval=10, blit=False)
        self.fig.subplots_adjust(hspace=0, wspace=0)

        self.input_mics = np.zeros((self.x_len, self.no_of_mics))
        #print(self.input_mics) 

        # which miro
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
       
        # subscribers
        self.sub_mics = rospy.Subscriber(topic_base_name + "/sensors/mics",
            Int16MultiArray, self.callback_mics, queue_size=1, tcp_nodelay=True)

        # publishers
        self.pub_push = rospy.Publisher(topic_base_name + "/core/mpg/push", miro.msg.push, queue_size=0)
        self.pub_wheels = rospy.Publisher(topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0)

        # prepare push message
        self.msg_push = miro.msg.push()
        self.msg_push.link = miro.constants.LINK_HEAD
        self.msg_push.flags = (miro.constants.PUSH_FLAG_NO_TRANSLATION + miro.constants.PUSH_FLAG_VELOCITY)

        # status flags
        self.audio_event = None
        self.orienting = False
        self.action_time = 1 #secs
        self.thresh = 0.05

        # time
        self.frame_p = None
        self.msg_wheels = TwistStamped()
        self.controller = miro.lib.PoseController()
        self.cmd_vel = miro.lib.DeltaPose()

        # save previous head data
        self.tmp = []
        # dynamic threshold
        self.thresh = 0
        self.thresh_min = 0.03

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        #msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        self.msg_wheels.twist.linear.x = dr
        self.msg_wheels.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.pub_wheels.publish(self.msg_wheels)
    

    def callback_mics(self, data):
        # data for angular calculation
        self.audio_event = AudioEng.process_data(data.data)

        # data for dynamic thresholding
        data_t = np.asarray(data.data, 'float32') * (1.0 / 32768.0)
        data_t = data_t.reshape((4, 500))    
        self.head_data = data_t[2][:]
        if self.tmp is None:
            self.tmp = np.hstack((self.tmp, np.abs(self.head_data)))
        elif (len(self.tmp)<10500):
            self.tmp = np.hstack((self.tmp, np.abs(self.head_data)))
        else:
            # when the buffer is full
            self.tmp = np.hstack((self.tmp[-10000:], np.abs(self.head_data)))
            # dynamic threshold is calculated and updated when new signal come
            self.thresh = self.thresh_min + AudioEng.non_silence_thresh(self.tmp)

        # data for display
        data = np.asarray(data.data)
        # 500 samples from each mics
        data = np.transpose(data.reshape((self.no_of_mics, 500)))
        data = np.flipud(data)
        self.input_mics = np.vstack((data, self.input_mics[:self.x_len-500,:]))

    
    def voice_accident(self):
        m = 0.00
        if self.audio_event != []:
            if self.audio_event != None:
                if self.audio_event[0] != None:
                    ae = self.audio_event[0]
                    #print(self.audio_event[2])
                    #print("Azimuth: {:.2f}; Elevation: {:.2f}; Level : {:.2f}".format(ae.azim, ae.elev, ae.level))
                    self.frame = self.audio_event[1]
                    m = (self.audio_event[2][0]+self.audio_event[2][1])/2
                    if m >= self.thresh:
                        self.status_code = 2
                    else:
                        self.status_code = 0
                else:
                    self.status_code = 0 
            else:
                self.status_code = 0 
        else:
            self.status_code = 0 

    def lock_onto_sound(self,ae_head):
        
        # detect if it is the frame within the same event
        # error may occur when the sound source is nearby the 90 degree of each side of MiRo
        # this is because the sample rate is not high enough for calculating in a higher accuracy
        # thus MiRo may considerred the angel inside (85,95) or (-95,-85) as the same angel
        if ae_head.x == self.frame_p:
            self.status_code = 0 

        else:
            # the frame is different: not from the same event
            self.frame_p = ae_head.x
            self.turn_to_sound()
            print("MiRo is moving......")
            self.status_code = 0 


    def turn_to_sound(self): 
        if self.audio_event[0] is None:
            return
        print("angular in degrees:{:.2f}".format(self.audio_event[0].ang))
        v = self.audio_event[0].azim
        #MiRo finish its rotation in 0.5s
        Tf = 0.5
        T1=0
        while(T1 <= Tf):

            #self.drive(v*2,v*2)
            self.msg_wheels.twist.linear.x = 0.0
            self.msg_wheels.twist.angular.z = v*2

            # test output
            #self.msg_wheels.twist.angular.z = 0.0

            self.pub_wheels.publish(self.msg_wheels)
            time.sleep(0.02)
            T1+=0.02


    def loop(self):
        msg_wheels = TwistStamped()

        # This switch loops through MiRo behaviours:
        # Listen to sound, turn to the sound source
        self.status_code = 0
        while not rospy.core.is_shutdown():

            # Step 1. sound event detection
            if self.status_code == 1:
                # Every once in a while, look for ball
               self.voice_accident()

            # Step 2. Orient towards it
            elif self.status_code == 2:
                self.lock_onto_sound(self.frame)
                #clear the data collected when miro is turning
                self.audio_event=[]


            # Fall back
            else:
                self.status_code = 1


    def update_line(self, i, left_ear_ys, right_ear_ys, head_ys, tail_ys):
        #Flip buffer so that incoming data moves in from the right
        left_ear_data = np.flipud(self.input_mics[:, 0])
        right_ear_data = np.flipud(self.input_mics[:, 1])
        head_data = np.flipud(self.input_mics[:, 2])
        tail_data = np.flipud(self.input_mics[:, 3])

        #Append new buffer data to plotting data
        left_ear_ys = np.append(left_ear_ys, left_ear_data)
        right_ear_ys = np.append(right_ear_ys, right_ear_data)
        head_ys = np.append(head_ys, head_data)
        tail_ys = np.append(tail_ys, tail_data)

        #Remove old sample outside of plot
        left_ear_ys = left_ear_ys[-self.x_len:]
        right_ear_ys = right_ear_ys[-self.x_len:]
        head_ys = head_ys[-self.x_len:]
        tail_ys = tail_ys[-self.x_len:]

        #Set data to line
        self.left_ear_line.set_ydata(left_ear_ys)
        self.right_ear_line.set_ydata(right_ear_ys)
        self.head_line.set_ydata(head_ys)
        self.tail_line.set_ydata(tail_ys)

        #Return the line to be animated
        return self.left_ear_line, self.right_ear_line, self.head_line, self.tail_line,

    def animation_init(self):
        self.left_ear_line.set_ydata(np.zeros(self.x_len))
        self.right_ear_line.set_ydata(np.zeros(self.x_len))
        self.head_line.set_ydata(np.zeros(self.x_len))
        self.tail_line.set_ydata(np.zeros(self.x_len))
        return self.left_ear_line, self.right_ear_line, self.head_line, self.tail_line,


      

if __name__ == "__main__":

    rospy.init_node("point_to_sound", anonymous=True)
    AudioEng = DetectAudioEngine()
    main = AudioClient()
    #plt.show() # to stop signal display next run: comment this line and line 89(self.ani...)
    main.loop()