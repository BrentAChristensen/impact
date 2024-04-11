#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class sensor_checking:

    def __init__(self):
        sub_topic_name = "/simple_camera/image_raw"
        self.camera_subscriber = rospy.Subscriber(sub_topic_name,Image,self.camera_cb)
        self.bridge = CvBridge()

    def camera_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data)
        print(frame.shape)
        cv2.imshow("output",frame)
        cv2.waitKey(1)


if __name__=='__main__':
    node_name="sensor_check"
    rospy.init_node(node_name)
    sensor_checking()
    rospy.spin()
        