#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from moveit_py.robot_interface import RobotCommander
from moveit_py.planning_interface import PlanningSceneInterface, MoveGroupCommander

class MoveGroupPythonInterface(Node):

    def __init__(self):
        super().__init__('move_group_python_interface')
        
      # Initialize `rclpy`
        rclpy.init(args=sys.argv)
        
        # Create a RobotCommander object.
        self.robot = RobotCommander()

        # Create MoveGroupCommander objects for arm and gripper groups
        self.group_name_arm = "arm_group"
        self.group_name_gripper = "gripper_group"
        self.move_group_arm = MoveGroupCommander(self.group_name_arm)
        self.move_group_gripper = MoveGroupCommander(self.group_name_gripper)

        # Publisher for displaying planned paths
        self.display_trajectory_publisher = self.create_publisher(moveit_msgs.msg.DisplayTrajectory, '/move_group/display_planned_path', 20)

        # Get planning frames
        planning_frame_arm = self.move_group_arm.get_planning_frame()
        planning_frame_gripper = self.move_group_gripper.get_planning_frame()
        print(f"============ Planning frame (Arm): {planning_frame_arm}")
        print(f"============ Planning frame (Gripper): {planning_frame_gripper}")

        # Get end-effector link
        eef_link = self.move_group_arm.get_end_effector_link()
        print(f"============ End effector link: {eef_link}")

        # Get available planning groups
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", group_names)

        # Print robot state
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        # Get current joint values and set new goals
        joint_goal = self.move_group_arm.get_current_joint_values()
        joint_goal[0] = -0.5
        joint_goal[1] = 0.1
        joint_goal[2] = 0.1
        joint_goal[3] = -0.2
        joint_goal[4] = 0
        joint_goal[5] = -0.5

        joint_gripper_goal = self.move_group_gripper.get_current_joint_values()
        print(f"Gripper position: {joint_gripper_goal[0]}")
        joint_gripper_goal[1] = 0.7

        # Execute the planned motions
        self.move_group_arm.go(joint_goal, wait=True)
        self.move_group_arm.stop()  # Ensure to stop any residual movement
        self.move_group_arm.clear_pose_targets()  # Clear targets after planning with poses

        print(f"Gripper position: {joint_gripper_goal[0]}")
        self.move_group_gripper.go(joint_gripper_goal, wait=True)
        self.move_group_gripper.stop()  # Ensure to stop any residual movement
        self.move_group_gripper.clear_pose_targets()  # Clear targets after planning with poses
        print(f"Gripper position: {joint_gripper_goal[0]}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveGroupPythonInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()