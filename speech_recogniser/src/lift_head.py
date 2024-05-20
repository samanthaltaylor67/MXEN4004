#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import math
import os

class lift_head:
    def __init__(self):
        # Initialise the lift_head node
        rospy.init_node('lift_head', anonymous=True)
        rospy.Subscriber("/speech_recogniser/start", Bool, self.callback_start)
        rospy.Subscriber('/speech_recogniser/end_conversation', Bool, self.callback_killall)

        # Get default robot name from environment
        self.robot_name = os.getenv("MIRO_ROBOT_NAME")

        # Check that robot name has been correctly set
        assert self.robot_name is not None, "MIRO_ROBOT_NAME environment variable has\
            					not been set. Please set it or specify robot_name"

        topic_base_name = "/" + self.robot_name

        # Initialise joints
        self.tilt, self.lift, self.yaw, self.pitch = range(4)
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(34), 0.0, 0.0]

        # Publisher
        self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)

    def callback_killall(self, data):
        # kill recogniser node
        if data.data:
            self.kin_joints.position[self.lift] = math.radians(56.1)
            self.pub_kin.publish(self.kin_joints)
            rospy.signal_shutdown("Conversation Ended")

    def callback_start(self, data):
        # Lift head when conversation begins
        if data.data:
            self.kin_joints.position[self.lift] = math.radians(14)
            self.pub_kin.publish(self.kin_joints)

if __name__ == "__main__":
    lift_head = lift_head()
    rospy.spin()