#!/usr/bin/env python2

import rospy
from std_msgs.msg import Empty, String
from threading import Thread, Lock
import sys



#roslaunch wheeltec_gripper_hardware_interface wheeltec_gripper_hardware_interface.launch
class Gripper:
    openpub = rospy.Publisher
    closepub = rospy.Publisher
    statsub = rospy.Subscriber
    state  = String()


    def __init__(self):
        rospy.init_node('gripper_node')
        self.openpub = rospy.Publisher('green', Empty, queue_size=1)
        self.closepub = rospy.Publisher('red', Empty, queue_size=1)
        rospy.Subscriber('state', String, self.State)


    def close(self):
      empty_msg = Empty()
      self.closepub.publish(empty_msg)
      rospy.loginfo("Published gripper close")
      rospy.sleep(1)
                

    def open(self):
        open_msg = Empty()
        self.openpub.publish(open_msg)
        rospy.loginfo("Published gripper open")
        rospy.sleep(1)

    def State(self,data):
        rospy.loginfo(data.data)
        if data.data == "The Green LED Blinks!":
           self.state = "open"
        else:
           self.state = "close"
           rospy.sleep(5)
    
    def Spin(self):
       rospy.spin()

'''def main(args):
    gripper = Gripper()
    rospy.init_node('gripper_node')
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")   
    
if __name__ == '__main__':
    main(sys.argv)
    try:
         while not rospy.is_shutdown():
            print('Enter command')
            x = input()
            if x == 'open':
                gripper.open()
            else:
                gripper.close()
            
            
            
   
        
    except rospy.ROSInterruptException:
        pass'''
            
            

