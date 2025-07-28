#!/usr/bin/env python3 

import rospy
import moveit_commander
import sys
import time # Import time for potential pauses

def move_through_named_poses():
    # 1. Initialize ROS and MoveIt! Commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_poses', anonymous=True)

    rospy.loginfo("Initializing MoveIt! Commander...")

    # RobotCommander: Provides information about the robot as a whole
    robot = moveit_commander.RobotCommander()

    # PlanningSceneInterface: For interacting with the world/objects (not strictly needed for just moving named poses, but good practice)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1) # Give the scene time to connect

    # MoveGroupCommander: For planning and executing movements for specific groups
    arm_group_name = "Arm_group"
    ee_group_name = "EE_group"
    move_group_arm = moveit_commander.MoveGroupCommander(arm_group_name)
    move_group_ee = moveit_commander.MoveGroupCommander(ee_group_name)

    rospy.loginfo("Robot Groups: %s", robot.get_group_names())
    rospy.loginfo("Arm Planning Frame: %s", move_group_arm.get_planning_frame())
    #rospy.loginfo("End Effector Planning Frame: %s", move_group_ee.get_planning_frame()) # EE group might not have a planning frame if not used for Cartesian planning

    # Get the list of available named states (optional, for verification)
    rospy.loginfo("Available Arm Named States: %s", move_group_arm.get_named_targets())
    rospy.loginfo("Available EE Named States: %s", move_group_ee.get_named_targets())


    # --- Define the sequence of named poses ---
    arm_sequence = ["pose0","pre_pick", "pick_pose", "place_rock","pose0"]
    # The gripper states are controlled separately at specific points

    #Example: Set a joint tolerance of 0.01 radians (approx 0.57 degrees)
    custom_joint_tolerance = 0.001 
    move_group_arm.set_goal_tolerance(custom_joint_tolerance)
    rospy.loginfo(f"Set joint goal tolerance to: {custom_joint_tolerance} radians.")

    # --- Execute the sequence ---

    rospy.loginfo("Starting sequence of movements through named poses...")

    for target_pose_name in arm_sequence:
        rospy.loginfo("Moving Arm_group to pose: %s", target_pose_name)

        # Set the target to the named pose
        move_group_arm.set_named_target(target_pose_name)

        # Plan and execute the movement
        # Using go() is suitable for moving to named targets
        success = move_group_arm.go(wait=True)

        # Check if the movement was successful
        if success:
            rospy.loginfo("Successfully moved to %s.", target_pose_name)
        else:
            rospy.loginfo("Approximately moved to %s.", target_pose_name)
            move_group_arm.stop()


        move_group_arm.stop() # Ensure no residual movement commands
        # move_group_arm.clear_pose_targets() # Clearing pose targets is more for random/explicit poses, less critical for named targets

        # --- Control the end effector at specific points ---
        if target_pose_name == "pre_pick":
            # After reaching straightup_pose, open the end effector
            rospy.loginfo("Reached pre_pick pose. Opening end effector...")
            ee_state_name = "ee_open"
            move_group_ee.set_named_target(ee_state_name)
            success_ee = move_group_ee.go(wait=True)
            if success_ee:
                rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            else:
                 rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            move_group_ee.stop()
            # Optional: Add a short pause
            time.sleep(1.0)
        elif target_pose_name == "pick_pose":
            # After reaching straightup_pose, open the end effector
            rospy.loginfo("Reached pick_pose. Grasping the object...")
            ee_state_name = "ee_close"
            move_group_ee.set_named_target(ee_state_name)
            success_ee = move_group_ee.go(wait=True)
            if success_ee:
                rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            else:
                 rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            move_group_ee.stop()
            # Optional: Add a short pause
            time.sleep(1.0)
        elif target_pose_name == "place_rock":
            # After reaching straightup_pose, open the end effector
            rospy.loginfo("Reached plase_pose. opening the gripper...")
            ee_state_name = "ee_open"
            move_group_ee.set_named_target(ee_state_name)
            success_ee = move_group_ee.go(wait=True)
            if success_ee:
                rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            else:
                 rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            move_group_ee.stop()
            # Optional: Add a short pause
            time.sleep(1.0)
        elif target_pose_name == "pose0":
            # After reaching straightup_pose, open the end effector
            rospy.loginfo("Reached plase_pose. opening the gripper...")
            ee_state_name = "ee_close"
            move_group_ee.set_named_target(ee_state_name)
            success_ee = move_group_ee.go(wait=True)
            if success_ee:
                rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            else:
                 rospy.loginfo("Successfully set end effector to %s.", ee_state_name)
            move_group_ee.stop()
            # Optional: Add a short pause
            time.sleep(1.0)

    rospy.loginfo("Sequence finished.")

    # --- Shutdown ---
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    # try:
        # Ensure Gazebo and Rviz with MoveIt are already launched
    move_through_named_poses()
    # except rospy.ROSInterruptException:
    #     pass