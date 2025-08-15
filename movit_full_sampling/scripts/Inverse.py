#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
import math
from geometry_msgs.msg import Pose, PoseStamped, Quaternion # For defining Cartesian poses
from tf.transformations import quaternion_from_euler, euler_from_quaternion # For creating quaternions

def create_pose(x, y, z, roll_rad, pitch_rad, yaw_rad):
    """
    Creates a geometry_msgs.msg.Pose object from XYZ coordinates and Roll, Pitch, Yaw angles.

    Args:
        x (float): X-coordinate of the position.
        y (float): Y-coordinate of the position.
        z (float): Z-coordinate of the position.
        roll_rad (float): Roll angle in radians.
        pitch_rad (float): Pitch angle in radians.
        yaw_rad (float): Yaw angle in radians.

    Returns:
        geometry_msgs.msg.Pose: The populated Pose message.
    """
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z


    # Convert Euler angles (radians) to a quaternion
    # quaternion_from_euler returns (x, y, z, w)
    qx, qy, qz, qw = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

    # Set the orientation of the Pose message
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw

    return target_pose


def move_arm_to_sequential_xyz_poses():
    # 1. Initialize ROS and MoveIt! Commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_xyz_poses', anonymous=True)

    rospy.loginfo("Initializing MoveIt! Commander...")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface() 
    rospy.sleep(2)

    arm_group_name = "Arm_group"
    ee_group_name = "EE_group" # Your end-effector group name
    move_group_arm = moveit_commander.MoveGroupCommander(arm_group_name)
    move_group_ee = moveit_commander.MoveGroupCommander(ee_group_name) # Commander for gripper

    # --- Set the end-effector link for planning ---
    eef_link = "link_6"
    move_group_arm.set_end_effector_link(eef_link)
    end_effector_link = move_group_arm.get_end_effector_link()
    rospy.loginfo("Using End Effector Link for planning: %s", move_group_arm.get_end_effector_link())

    rospy.loginfo("Robot Groups: %s", robot.get_group_names())
    rospy.loginfo("Arm Planning Frame: %s", move_group_arm.get_planning_frame())
    rospy.loginfo("Current end-effector pose:\n%s", move_group_arm.get_current_pose().pose)

    # Maintaining current orientation for all moves of end effector.
    target_orientation = move_group_arm.get_current_pose().pose.orientation 
    # Convert the current Quaternion to Euler angles (radians)
    (roll_goal, pitch_goal, yaw_goal) = euler_from_quaternion([target_orientation.x,
                                                                          target_orientation.y,
                                                                          target_orientation.z,
                                                                          target_orientation.w])

    # Example: Set a joint tolerance of 0.001 radians
    custom_joint_tolerance = 0.001
    move_group_arm.set_goal_tolerance(custom_joint_tolerance)
    rospy.loginfo(f"Set joint goal tolerance to: {custom_joint_tolerance} radians.")

    target_xyz_rpy_points = [
        (0.0 , 0.823 , 0.095 , roll_goal , pitch_goal , yaw_goal), # index 0: first point
        (0, 0.82, 0.09, 3.12 , -0.06, -1.59), # index 1: second point
        (-0.001682, 0.823266, 0.095203, 3.116725 , -0.055879 , -1.592038), # index 2: third point
        (0.0, 0.823, 0.095, 3.115 , -0.056 , 1)
    ]

    rospy.loginfo("Defined %d sequential XYZ poses.", len(target_xyz_rpy_points))

    # --- Execute the sequence of movements ---
    for i, (x, y, z,roll,pitch,yaw) in enumerate(target_xyz_rpy_points):
        if rospy.is_shutdown(): # Check for shutdown before each new move
            rospy.loginfo("ROS Shutdown detected. Aborting sequence.")
            break
        
        # Ensure you are moving from pose0 
        rospy.loginfo("\nAttempting to move to pose0.")
        move_group_arm.set_named_target("pose0")
        success = move_group_arm.go(wait=True)
        if success:
            rospy.loginfo("Successfully moved to pose0.")
        else:
            rospy.logwarn("Approximately moved to Pose0")
            move_group_arm.stop()

        # opening griper before moving to target pick points
        rospy.loginfo(" Opening gripper.")
        move_group_ee.set_named_target("ee_open")
        success_ee_open = move_group_ee.go(wait=True)
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # Use rospy.sleep


        # Log the target pose details
        rospy.loginfo(f"\n--- Moving to Pose {i+1}: (x={x:.3f}, y={y:.3f}, z={z:.3f}) ---")

        pose_goal = create_pose(x, y, z, roll, pitch, yaw)
        move_group_arm.set_pose_target(pose_goal)

        move_group_arm.set_planning_time(10.0)

        rospy.loginfo("Planning trajectory...")
        plan_result = move_group_arm.plan()
        success = plan_result[0]
        plan_trajectory = plan_result[1]

        if not success or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory for Pose %d. Skipping.", i+1)
            rospy.logwarn("Tip: Ensure the target pose is within the robot's reachable workspace and collision-free.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
            continue
        else:
            rospy.loginfo("Planning successful. Executing trajectory...")
            # Execute with a check for shutdown during execution
            execute_success = move_group_arm.execute(plan_trajectory, wait=True)

            # If Ctrl+C is pressed during execute, it might return false or raise an exception
            if rospy.is_shutdown(): # Check again after execution attempt
                rospy.loginfo("ROS Shutdown detected during execution. Stopping.")
                move_group_arm.stop()
                break

            if execute_success:
                rospy.loginfo("Successfully executed trajectory to Pose %d.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final end-effector pose:\n%s", final_pose)
                rospy.sleep(1.0) # Use rospy.sleep instead of time.sleep
            else:
                rospy.logwarn("Trajectory execution failed for Pose %d. This might be due to not reaching tolerance.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final approximated end-effector pose:\n%s", final_pose)
                rospy.sleep(1.0) # Use rospy.sleep instead of time.sleep

            move_group_arm.stop()
            move_group_arm.clear_pose_targets()

        # --- GRIPPER CONTROL LOGIC ---
        if rospy.is_shutdown(): # Check before gripper action
            rospy.loginfo("ROS Shutdown detected. Aborting gripper action.")
            break

        
        rospy.loginfo("At first point. Opening gripper.")
        move_group_ee.set_named_target("ee_close")
        success_ee_open = move_group_ee.go(wait=True)
        if success_ee_open:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # Use rospy.sleep



    # After the loop, return to a predefined 'pose0'
    if not rospy.is_shutdown(): # Only attempt if not shutting down
        rospy.loginfo("\nAttempting to move to pose0.")
        move_group_arm.set_named_target("pose0")
        success = move_group_arm.go(wait=True)
        if success:
            rospy.loginfo("Successfully moved to pose0.")
        else:
            rospy.logwarn("Approximately moved to Pose0")
            move_group_arm.stop()
    else:
        rospy.loginfo("Skipping return to pose0 due to shutdown.")

    move_group_arm.stop() # Ensure arm is stopped on shutdown

    rospy.loginfo("\nFinished or interrupted moving through sequential XYZ poses.")

    # --- Shutdown ---
    rospy.loginfo("Shutting down MoveIt! Commander.")
    # moveit_commander.roscpp_shutdown() is typically called automatically when rospy.init_node goes out of scope
    # or when the ROS node fully shuts down, but explicitly calling it here is fine.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        # Ensure Gazebo and Rviz with MoveIt are already launched
        move_arm_to_sequential_xyz_poses()
    except rospy.ROSInterruptException:
        # This exception is caught when Ctrl+C is pressed and rospy signals shutdown.
        rospy.loginfo("ROS Interrupt received. Shutting down node.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", str(e))
    finally:
        # Any cleanup code that must run regardless of shutdown or error
        # moveit_commander.roscpp_shutdown() is generally called here if not already done
        pass