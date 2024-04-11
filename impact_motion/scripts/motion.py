#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()   
group_name_arm = "arm_group"
group_name_gripper = "gripper_group"
move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame_arm = move_group_arm.get_planning_frame()
planning_frame_gripper = move_group_gripper.get_planning_frame()
print ("============ Planning frame: %s" % planning_frame_arm)

# We can also print the name of the end-effector link for this group:
eef_link = move_group_arm.get_end_effector_link()
print ("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Available Planning Groups:"), robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print ("============ Printing robot state")


print (robot.get_current_state())
print ("")

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group_arm.get_current_joint_values()
joint_goal[0] = -0.5
joint_goal[1] = 0.1
joint_goal[2] = 0.1
joint_goal[3] = -0.2
joint_goal[4] = 0
joint_goal[5] = -0.5


joint_gripper_goal = move_group_gripper.get_current_joint_values()
print ("gripper position:" +str(joint_gripper_goal[0]))
joint_gripper_goal[1] = 0.7
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group_arm.go(joint_goal, wait=True)
print ("gripper position:" +str(joint_gripper_goal[0]))
move_group_gripper.go(joint_gripper_goal, wait=True)
print ("gripper position:" +str(joint_gripper_goal[0]))
'''
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = w
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)


plan = move_group.go(wait=True)
'''
# Calling `stop()` ensures that there is no residual movement
move_group_arm.stop()
move_group_gripper.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group_arm.clear_pose_targets()
move_group_gripper.clear_pose_targets()
rospy.sleep(5)

moveit_commander.roscpp_shutdown()