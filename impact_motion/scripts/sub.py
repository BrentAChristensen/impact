#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#rosrun rosserial_python serial_node.py _port:=COM3 _baud:=9600

class gripper_subscriber(object):
    def __init__(self):
        # create ros subscriber on command topic
        self._cmd_sub = rospy.Subscriber(
            "/state", String, self.callback,queue_size=50)
       
    def callback(ros_msg):
        print(ros_msg.data)

if __name__ == '__main__':
    rospy.init_node('gripper_subscriber_node')
    rospy.loginfo("gripper_subscriber_node started ...")
    # Instantiate object
    gripper_subscriber = gripper_subscriber()
    # spin the node
    rospy.spin()