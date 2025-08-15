#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
# No need for math, quaternion_from_euler, euler_from_quaternion as we don't care about specific orientation input
from geometry_msgs.msg import Pose, PoseStamped, Quaternion # Keep these types for general pose info

def move_arm_to_sequential_xyz():
    """
    Initializes MoveIt! and moves the robotic arm's end-effector (link_5)
    through a sequence of XYZ target positions. The orientation of the
    end-effector will be determined by the Inverse Kinematics solver,
    allowing maximum flexibility to reach the given position.
    Includes gripper control logic and graceful shutdown handling.
    """
    # 1. Initialize ROS and MoveIt! Commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_xyz', anonymous=True)

    rospy.loginfo("Initializing MoveIt! Commander...")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2) # Give MoveIt time to connect to the planning scene

    arm_group_name = "Arm_group"
    ee_group_name = "EE_group" # Your end-effector group name (gripper)
    move_group_arm = moveit_commander.MoveGroupCommander(arm_group_name)
    move_group_ee = moveit_commander.MoveGroupCommander(ee_group_name) # Commander for gripper

    # --- Set the end-effector link for planning ---
    # This is crucial: it tells MoveIt which link's XYZ position to control
    eef_link = "link_5"
    move_group_arm.set_end_effector_link(eef_link)
    rospy.loginfo("Using End Effector Link for planning: %s", move_group_arm.get_end_effector_link())

    rospy.loginfo("Robot Groups: %s", robot.get_group_names())
    rospy.loginfo("Arm Planning Frame: %s", move_group_arm.get_planning_frame())
    rospy.loginfo("Current end-effector pose:\n%s", move_group_arm.get_current_pose().pose)

    # Set a joint tolerance for reaching goals
    custom_joint_tolerance = 0.001 # in radians
    move_group_arm.set_goal_tolerance(custom_joint_tolerance)
    rospy.loginfo(f"Set joint goal tolerance to: {custom_joint_tolerance} radians.")

    # Define sequential XYZ points 
    target_xyz_points = [
        (0.82, 0, 0.3), # Example Point to pick rock 1
        (0.8 , 0.25 , 0.4 ), # Example Point to pick rock 2
        (0.9 , -0.13 , 0.35 ) # Example Point to pick rock 3
    ]
    
    rospy.loginfo("Defined %d sequential XYZ points (orientation will be flexible).", len(target_xyz_points))
    
    # Ensure arm returns to a known 'pose0' at start of script
    rospy.loginfo("\nAttempting to move to pose0 (home/start position).")
    move_group_arm.set_named_target("pose0") # Make sure "pose0" is defined in your SRDF or MoveIt config
    success_home_start = move_group_arm.go(wait=True)
    if success_home_start:
        rospy.loginfo("Successfully moved to pose0.")
    else:
        rospy.logwarn("Approximately moved to Pose0 or failed to reach. Continuing anyway.")
        move_group_arm.stop() # Stop any residual motion


    # --- Execute the sequence of movements ---
    for i, (x, y, z) in enumerate(target_xyz_points):
        if rospy.is_shutdown():
            rospy.loginfo("ROS Shutdown detected. Aborting sequence.")
            break
        
        # move to pre_pick pose before each target
        rospy.loginfo("\nAttempting to move to pre pick position.")
        move_group_arm.set_named_target("pre_pick") # Make sure "pre_pick" is defined in your SRDF or MoveIt config
        success_pre_pick = move_group_arm.go(wait=True)
        if success_pre_pick:
            rospy.loginfo("Successfully moved to pre pose.")
        else:
            rospy.logwarn("Approximately moved to Pre pose or failed to reach. Continuing anyway.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper before moving to a new target to grasp the rocks
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") # Make sure "ee_open" is defined for your gripper group
        success_ee_open = move_group_ee.go(wait=True)
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep

        # Log the current target pose details
        rospy.loginfo(f"\n--- Moving to Target {i+1}: (x={x:.3f}, y={y:.3f}, z={z:.3f}) ---")

        # *** THE KEY CHANGE: Use set_position_target() ***
        move_group_arm.set_position_target([x, y, z]) # Provide target position as a list/tuple

        # Set planning time (important for complex plans)
        move_group_arm.set_planning_time(5.0)

        rospy.loginfo("Planning trajectory...")
        plan_result = move_group_arm.plan()
        success_plan = plan_result[0]
        plan_trajectory = plan_result[1]

        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory for Target %d. Skipping.", i+1)
            rospy.logwarn("Tip: Ensure the target XYZ position is within the robot's reachable workspace and collision-free.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets() # Clear any remaining targets
            continue
        else:
            rospy.loginfo("Planning successful. Executing trajectory...")
            # Access the trajectory's waypoints
            original_points = plan_trajectory.joint_trajectory.points
            rospy.loginfo("Original trajectory has %d waypoints.", len(original_points))
            # Execute the planned trajectory
            execute_success = move_group_arm.execute(plan_trajectory, wait=True)

            if rospy.is_shutdown():
                rospy.loginfo("ROS Shutdown detected during execution. Stopping.")
                move_group_arm.stop()
                break

            if execute_success:
                rospy.loginfo("Successfully executed trajectory to Target %d.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final end-effector pose (position and ACHIEVED orientation):\n%s", final_pose)
                rospy.sleep(1.0)
            else:
                rospy.logwarn("Trajectory execution failed for Target %d. This might be due to not reaching tolerance.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final approximated end-effector pose:\n%s", final_pose)
                rospy.sleep(1.0)
            
            # Clear target after each move to ensure planner re-evaluates next goal fresh
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()


        # Close gripper after reaching a target (e.g., after "grasping" something at the location)
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") # Make sure "ee_close" is defined for your gripper group
        success_ee_close = move_group_ee.go(wait=True)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep


        rospy.loginfo("\nAttempting to move to place rock pose.")
        move_group_arm.set_named_target("place_rock") # Make sure "place_rock" is defined in your SRDF 
        success_place = move_group_arm.go(wait=True)
        if success_place:
            rospy.loginfo("Successfully moved to place rock.")
        else:
            rospy.logwarn("Approximately moved to place rock.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper to drop the rock
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") 
        success_ee_open = move_group_ee.go(wait=True)
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

        # Close gripper after dropping the rock
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") 
        success_ee_close = move_group_ee.go(wait=True)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

    # After the loop, return to a predefined 'pose0' one last time for safety/reset
    if not rospy.is_shutdown():
        rospy.loginfo("\nAttempting final return to pose0.")
        move_group_arm.set_named_target("pose0")
        success_final_home = move_group_arm.go(wait=True)
        if success_final_home:
            rospy.loginfo("Successfully returned to pose0.")
        else:
            rospy.logwarn("Approximately returned to Pose0 or failed.")
            move_group_arm.stop()
    else:
        rospy.loginfo("Skipping final return to pose0 due to shutdown.")

    move_group_arm.stop() # Ensure arm is stopped on shutdown

    rospy.loginfo("\nFinished or interrupted moving through sequential XYZ poses.")

    # --- Shutdown ---
    rospy.loginfo("Shutting down MoveIt! Commander.")
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        # Ensure Gazebo and Rviz with MoveIt are already launched
        move_arm_to_sequential_xyz()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down node.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", str(e))
    finally:
        pass
