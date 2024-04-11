import rospy
from std_msgs.msg import Empty, String

rospy.init_node('node_name')
OurLedState = String()
LEDstate = rospy.Publisher('state', String, queue_size=10)
RedState = "The Red LED blinks!"
GreenState = "The Green LED Blinks!"

def RedOne(toggle_msg):
    rospy.digitalWrite(13, rospy.HIGH)
    rospy.delay(3)
    rospy.digitalWrite(13, rospy.LOW)
    OurLedState.data = RedState

def GreenOne(toggle_msg):
    rospy.digitalWrite(12, rospy.HIGH)
    rospy.delay(3)
    rospy.digitalWrite(12, rospy.LOW)
    OurLedState.data = GreenState

RedLED = rospy.Subscriber('red', Empty, RedOne)
GreenLED = rospy.Subscriber('green', Empty, GreenOne)

rospy.spin()

while not rospy.is_shutdown():
    LEDstate.publish(OurLedState)
    rospy.spinOnce()
    rospy.delay(0.001)
