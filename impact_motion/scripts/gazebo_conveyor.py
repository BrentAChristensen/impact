#!/usr/bin/env python3

import rospy
from gazebo_conveyor.srv import ConveyorBeltControl


class Conveyor():

    def __init__(self):
        rospy.wait_for_service('/conveyor/control')
        print('converyor control running')
        
    def conveyor_service_result(self,msg):
        print(msg)
        return msg

    def speed(self,value):
        try:
            conveyor_speed = rospy.ServiceProxy('/conveyor/control', ConveyorBeltControl)
            result = conveyor_speed(value)
            return 'true'
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

