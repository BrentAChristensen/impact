#!/usr/bin/env python3


import rospy
from moveit_commander import RobotCommander, MoveGroupCommander,PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import moveit_msgs
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion
import tf_conversions
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from threading import Timer
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String





class commander(MoveGroupCommander):
   Target = geometry_msgs.msg.Pose()

   def __init__(self, name, robot_description="robot_description", ns="", wait_for_servers=5):
       super().__init__(name, robot_description, ns, wait_for_servers)

   def target(self):
       return self.Target
   
   def set_joint(self,joint_name, joint_value):
        active_joints = self.get_current_joint_values()
        active_joint_names = self.get_active_joints()
        print(active_joint_names)
        index = active_joint_names.index(joint_name)
        print(str(index))
        active_joints[index] = joint_value
        self.go(active_joints, wait=True)

   def set_planner_id(self, planner_id):
       return super().set_planner_id(planner_id)
   
   def end_effector_link(self):
       return super().get_end_effector_link()
    
class Commanders():
     commanders = {} 

     def add(self,name):
         c = commander(name)
         self.commanders[name] = c
         
         rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
     
     def item(self,commander_name):        
         return self.commanders[commander_name]
     
     def count(self):
         i = 0
         for key in self.commanders:
              i = i +1
         return str(i)



class Plan():
    commanders = Commanders()
    end_effector = ""

    def __init__(self,*argv):        
       for arg in argv:
            self.commanders.add(arg)
       
    def get_yaw_pitch_roll(self,target_frame,source_frame):
         # **Assuming /tf2 topic is being broadcasted= 
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        transform = tf_buffer.lookup_transform(source_frame,
                                              target_frame,
                                              rospy.Time(0),
                                              rospy.Duration(1.0))
        
        rospy.loginfo('get_yaw_pitch_roll transform rotation x:' + format(transform.transform.rotation.x) + ' y:'+ format(transform.transform.rotation.y) + ' z:' + format(transform.transform.rotation.z) + ' w:' + format(transform.transform.rotation.w))
       
            
        quaterion = (transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(quaterion)

        roll,pitch,yaw = euler

        return yaw,pitch,roll



    def transform_pose(self,input_pose, target_frame, source_frame):

              
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        transform = tf_buffer.lookup_transform(target_frame,
                                              source_frame,
                                              rospy.Time(0),
                                              rospy.Duration(1.0))

        try:
           # poseStampedToTransform = commander.get_current_pose(commander.get_end_effector_link())
            poseStampedToTransform = geometry_msgs.msg.PoseStamped()


            pose_transformed = tf2_geometry_msgs.do_transform_pose(poseStampedToTransform, transform)
            object_pose = geometry_msgs.msg.PoseStamped()
 
            object_pose.pose.orientation.x = transform.transform.rotation.x
            object_pose.pose.orientation.y = transform.transform.rotation.y
            object_pose.pose.orientation.z = transform.transform.rotation.z
            object_pose.pose.orientation.w = transform.transform.rotation.w
            object_pose.pose.position.x = pose_transformed.pose.position.x
            object_pose.pose.position.y = pose_transformed.pose.position.y
            object_pose.pose.position.z = pose_transformed.pose.position.z
        
            return object_pose
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('not found')     
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return
            
        
    def print_end_effector_pose(self,commander_name):

        #Get the current position of end effector link
        pose_values = self.commanders.item(commander_name).get_current_pose().pose
        rospy.loginfo('x:' + format(pose_values.orientation.x) + ' y:'+ format(pose_values.orientation.y) + ' z:' + format(pose_values.orientation.z) + ' w:' + format(pose_values.orientation.w))
        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w
    

        print("The pos orientation is = " +str(q_w))

        #Store the quaternion position values in list
        quaternion_list = [q_x, q_y, q_z, q_w]
        #Covert the quaternion values to roll, pitch and yaw
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        #Print the values
        rospy.loginfo('\033[32m' + 
                                "\n" + "End-Effector ({}) Pose: \n\n".format(self.commanders.item(commander_name).end_effector_link()) + 
                                "x: {}\n".format(pose_values.position.x) +  "y: {}\n".format(pose_values.position.y) +    "z: {}\n\n".format(pose_values.position.z) + 
                                "roll: {}\n".format(roll) + "pitch: {}\n".format(pitch) + "yaw: {}\n".format(yaw) +
                                '\033[0m')    

    
    
    def Pose(self, commander_name, pose_name=None, x=None, y=None, z=None, R=None, P=None, Y=None, Joint_positions=None):
        rospy.loginfo("Starting pose with commmander " + commander_name)
        commander = self.commanders.item(commander_name)
        pose_goal = commander.target()
        if Joint_positions!=None:
           current_joints =  commander.get_current_joint_values()
           index =0
           for j in current_joints:
               current_joints[index] = Joint_positions[index]
               index = index+1
       
        if pose_name!=None:  
            commander.set_named_target(pose_name)
            commander.go(wait=True)
            commander.clear_pose_targets()            
        else:
            if x!=None and y!=None and z!=None and R!=None and P!=None and Y!=None:
                rospy.loginfo("setting XYZ for move group")
                commander.set_pose_target([x,y,z,R,P,Y],commander.get_end_effector_link())
                commander.go(wait=True)
                commander.clear_pose_targets()
                
            else:
                 rospy.loginfo("Starting position only move: ")
                 if commander.has_end_effector_link:
                    current_pose = PoseStamped()              
                    current_pose = commander.get_current_pose(commander.get_end_effector_link())
                    if x!=None:
                        current_pose.pose.position.x = current_pose.pose.position.x + x
                    if y!=None:
                        current_pose.pose.position.y = current_pose.pose.position.y + y
                    if z!=None:
                        rospy.loginfo("set z position")
                        current_pose.pose.position.z = current_pose.pose.position.z + z
                        print (current_pose.pose.position.z)
                    commander.set_pose_target(current_pose)
                    rospy.loginfo("executing set_position")
                    commander.go(wait=True)
                    commander.clear_pose_targets()
    

    def move_to_object(self,commander_name,object_name,offset_x=0,offset_y=0,offset_z=0):

        from_frame = "world"
        to_frame = object_name
        commander = self.commanders.item(commander_name)
        current_pose = commander.get_current_pose(commander.get_end_effector_link())
        goal_pose = self.transform_pose(current_pose,from_frame,to_frame)
        current_pose.pose.position.x = goal_pose.pose.position.x + offset_x
        current_pose.pose.position.y = goal_pose.pose.position.y + offset_y
        current_pose.pose.position.z = goal_pose.pose.position.z + offset_z
        current_pose.pose.orientation.w = 1.0
        #current_pose.pose.orientation.w = goal_pose.pose.orientation.w 
        #current_pose.pose.orientation.x = goal_pose.pose.orientation.x 
        #current_pose.pose.orientation.y = goal_pose.pose.orientation.y 
        #current_pose.pose.orientation.z = goal_pose.pose.orientation.z 
        rospy.loginfo('x:' + format(goal_pose.pose.orientation.x) + ' y:'+ format(goal_pose.pose.orientation.y) + ' z:' + format(goal_pose.pose.orientation.z) + ' w:' + format(goal_pose.pose.orientation.w))
        q_x = goal_pose.pose.orientation.x
        q_y = goal_pose.pose.orientation.y
        q_z = goal_pose.pose.orientation.z
        q_w = goal_pose.pose.orientation.w
    

        #Store the quaternion position values in list
        quaternion_list = [q_x, q_y, q_z, q_w]
        #Covert the quaternion values to roll, pitch and yaw
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        x = goal_pose.pose.position.x 
        y = goal_pose.pose.position.y
        z = goal_pose.pose.position.z
       
                #Print the values
        rospy.loginfo('\033[32m' + 
                                "\n" + "End-Effector ({}) Pose: \n\n".format(self.commanders.item(commander_name).end_effector_link()) + 
                                "x: {}\n".format(goal_pose.pose.position.x) +  "y: {}\n".format(goal_pose.pose.position.y) +    "z: {}\n\n".format(goal_pose.pose.position.z) + 
                                "roll: {}\n".format(roll) + "pitch: {}\n".format(pitch) + "yaw: {}\n".format(yaw) +
                                '\033[0m')    
        #self.Pose(commander_name,x=x,y=y,z=z,Y=yaw,P=pitch,R=roll)
        commander.set_pose_target(current_pose,commander.get_end_effector_link())
        result = commander.go(wait=True)
        return result






    
