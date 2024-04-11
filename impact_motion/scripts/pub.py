#!/usr/bin/env python
#rosrun rosserial_python serial_node.py _port:=COM3 _baud:=9600
#rostopic pub /gripper_command std_msgs/String 'close' -1
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String

class gripper_subscriber(object):
    gripper_state = String()
    def __init__(self):
        # create ros subscriber on command topic
        print("started gripper_subscriber")
        self._cmd_sub = rospy.Subscriber(
            "/state", String, self.callback,queue_size=50)
        rospy.spin()
       
    def callback(self,ros_msg):
        if ros_msg.data == "The Green LED Blinks!":
            self.gripper_state = 'open'
        elif ros_msg.data == "The Red LED blinks!":
            self.gripper_state = 'close'
        else:
            self.gripper_state = 'unknown'
        print (self.gripper_state)



class gripper_command(object):
    def __init__(self):
        # create publisher
        self._pub = rospy.Publisher(
            'green', Empty,queue_size=1)
        # create the msg
        self._msg = Empty()
        # create rate 50 hz
        self._freq = 50
        self._rate = rospy.Rate(self._freq)
        

    def run(self):
        while not rospy.is_shutdown():
            # publish message
            self._pub.publish(self._msg)
            self._rate.sleep()



if __name__ == '__main__':
    rospy.init_node('gripper_publisher_node')
    rospy.loginfo("gripper_publisher_node started ...")

    # Instantiate object
    gripper_pub_obj = gripper_command()
    gripper_sub_obj = gripper_subscriber()
    # run object
    gripper_pub_obj.run()
    # spin the node
    