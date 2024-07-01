import sys
import rospy
from moveit_commander import roscpp_initialize, roscpp_shutdown
from find_object_2d.msg import ObjectsStamped
import gazebo_conveyor
import tf2_ros
from time import sleep
from planning import Plan


class find_objects():
    objects = []

    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node('impact_motion', anonymous=True)
        self.conveyor = gazebo_conveyor()
        self.objFramePrefix = "object"
        self.targetFrameId = rospy.get_param("~target_frame_id", "ar3_base_link")
        self.objFramePrefix = rospy.get_param("~object_prefix", self.objFramePrefix)
        self.subs = rospy.Subscriber("objectsStamped", ObjectsStamped, self.objectsDetectedCallback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tf_buffer)

    def __del__(self):
        roscpp_shutdown()

    def objectsDetectedCallback(self,msg):
        
        if msg.objects.data:
            multiSubId = ord('b')
            previousId = -1
            for i in range(0, len(msg.objects.data), 12):
                # get data
                id = int(msg.objects.data[i])

                multiSuffix = ""
                if id == previousId:
                    multiSuffix = "" + chr(multiSubId)
                    multiSubId += 1
                else:
                    multiSubId = ord('b')
                previousId = id
                  # "object1", "object_1_b", "object_1_c", "object_2"
                objectFrameId = "{}{}{}".format(self.objFramePrefix, '_' + str(id), multiSuffix)
                if objectFrameId in self.objects:
                   # print(str(len(self.objects)))
                    pass
                else: 
                  print('adding object to list')
                  print (objectFrameId)
                  self.objects.append(objectFrameId)
                  sleep(1.65)
                  self.conveyor.speed(0)
                  sleep(2.0)
                  print("starting pickup code")
                  self.Pickup_object(objectFrameId)
                  
                # add each found object to an object collection

    def Pickup_object(self,object_name):
        c = gazebo_conveyor()
        arm = "arm_gripper_group"
        gripper = "gripper_group"
        ar3="arm_group"
        print("creating planning object")
        p = Plan(arm,gripper,ar3)
        _ = p
        print(p.commanders.item(arm).end_effector_link())
        
        try:
            _.Pose(gripper,"open")
            result =  _.move_to_object(arm,object_name,offset_y=-0.025, offset_z=0.04,offset_x=-0.05)
            if result == True:
                _.Pose(arm,z=-0.0485)
                _.Pose(arm,x=0.055)
                _.commanders.item(gripper).set_joint("left_finger_joint",0.545) 
                
            else:
                c.speed(15)
                _.Pose(ar3,"stand")
        except:
            c.speed(15)
            _.Pose(arm,"stand")
            self.objects.remove(object_name)

def Solution_Start():

        f = find_objects()

     
if __name__ == "__main__":
    try:
        Solution_Start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
