#!/usr/bin/env python3
#rosrun rosserial_python serial_node.py _port:=COM3 _baud:=9600
#rostopic pub /gripper_command std_msgs/String 'close' -1
import rospy
from std_msgs.msg import String,Float32
from trajectory_msgs.msg import JointTrajectory


class gripper_subscriber(object):
    gripper_state = String()
    def __init__(self):
        # create ros subscriber on command topic
        print("started gripper_subscriber")
        self._cmd_sub = rospy.Subscriber(
            "/wheeltec_gripper_controller/command", JointTrajectory, self.callback,queue_size=10)
        rospy.spin()
       
    def callback(self,ros_msg):
        self.gripper_state = ros_msg.points[0].positions[0]
        self._pub = self.gripper_state
        print (self.gripper_state)



class gripper_command(object):
    def __init__(self):
        # create publisher
        self._pub = rospy.Publisher(
            'wheeltec_gripper_position_requested', Float32,queue_size=10)
        # create the msg
        self._msg = 0.0
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
    