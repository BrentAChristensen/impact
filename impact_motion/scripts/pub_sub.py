#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
import numpy as np

pub = rospy.Publisher('/status', String, queue_size=1000)

def callback(data):
    print ("Message received")
    rospy.INFO("Message received")

def listener():

    rospy.init_node('control', anonymous=True)
    print ("listener function called")
    rospy.Subscriber('control_c', Empty, callback)
    rospy.spin()

if __name__ == '__main__':
    print ("Running")
    listener()