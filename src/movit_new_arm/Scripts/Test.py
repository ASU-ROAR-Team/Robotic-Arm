#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import time
import math
from geometry_msgs.msg import Pose, PoseStamped, Quaternion # For defining Cartesian poses
from tf.transformations import quaternion_from_euler # For creating quaternions

def create_pose_from_rpy(x, y, z, roll_deg, pitch_deg, yaw_deg):
    """
    Creates a geometry_msgs.msg.Pose object from XYZ coordinates and Roll, Pitch, Yaw angles.

    Args:
        x (float): X-coordinate of the position.
        y (float): Y-coordinate of the position.
        z (float): Z-coordinate of the position.
        roll_deg (float): Roll angle in degrees.
        pitch_deg (float): Pitch angle in degrees.
        yaw_deg (float): Yaw angle in degrees.

    Returns:
        geometry_msgs.msg.Pose: The populated Pose message.
    """
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    # Convert Euler angles from degrees to radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

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
    rospy.init_node('move_arm_to_sequential_xyz_poses', anonymous=True)

    rospy.loginfo("Initializing MoveIt! Commander...")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # Give the scene time to connect and for other MoveIt nodes to be ready
    rospy.sleep(2) 

    arm_group_name = "Arm_group"
    ee_group_name = "EE_group" # Your end-effector group name
    move_group_arm = moveit_commander.MoveGroupCommander(arm_group_name)
    move_group_ee = moveit_commander.MoveGroupCommander(ee_group_name) # Commander for gripper

    # --- Set the end-effector link for planning ---
    # Based on your URDF, link_6 is the appropriate end-effector link for the arm.
    eef_link = "link_5"
    move_group_arm.set_end_effector_link(eef_link)
    end_effector_link = move_group_arm.get_end_effector_link()
    rospy.loginfo("Using End Effector Link for planning: %s", move_group_arm.get_end_effector_link())

    rospy.loginfo("Robot Groups: %s", robot.get_group_names())
    rospy.loginfo("Arm Planning Frame: %s", move_group_arm.get_planning_frame())
    rospy.loginfo("Current end-effector pose:\n%s", move_group_arm.get_current_pose().pose)

    # Example: Set a joint tolerance of 0.001 radians
    custom_joint_tolerance = 0.001 
    move_group_arm.set_goal_tolerance(custom_joint_tolerance)
    rospy.loginfo(f"Set joint goal tolerance to: {custom_joint_tolerance} radians.")

    
    # Maintaining current orientation for all moves of end effector.
    # target_orientation = move_group_arm.get_current_pose().pose.orientation 

    
    # Box dimensions for planning scene
    box_size = (0.01, 0.01, 0.05) # x, y, z dimensions of the box
    box_name = "pickup_box" # Must match the model name in BOX_SDF_MODEL

    # Define the pick and place coordinates based on your list
    # Assuming the Z coordinate in your list is the *base* of the object + robot_base_offset
    PICK_X, PICK_Y= -0.6, 0 # From your target_xyz_points[2]
    PLACE_X, PLACE_Y, = 0.75, 0.0# From your target_xyz_points[4]


    
    target_xyz_rpy_points = [
        (0.257, -0.249, 0.533, 0 , 0 , 0), # index 0: first point
        (0.257, -0.337, 0.51, 0 , 0 , 0), # index 1: second point
    ]

    rospy.loginfo("Defined %d sequential XYZ poses.", len(target_xyz_rpy_points))

    # --- Add the box to MoveIt's planning scene ---
    # The box's Z position is its center, so it's PICK_Z_BASE + half_box_height
    box_pose_scene = PoseStamped()
    box_pose_scene.header.frame_id = "world"
    box_pose_scene.pose.position.x = PICK_X
    box_pose_scene.pose.position.y = PICK_Y
    box_pose_scene.pose.position.z = box_size[2] / 2.0 # Center of box
    box_pose_scene.pose.orientation.w = 1.0 # No rotation

    rospy.loginfo(f"Adding box '{box_name}' to MoveIt planning scene at ({box_pose_scene.pose.position.x}, {box_pose_scene.pose.position.y}, {box_pose_scene.pose.position.z})...")
    scene.add_box(box_name, box_pose_scene, box_size)
    rospy.sleep(1.0) # Give time for the object to be added


    # Get touch links for gripper (links that can touch the object when attached)
    grasping_group_links = robot.get_link_names(group=ee_group_name)
    rospy.loginfo(f"EE_group touch links: {grasping_group_links}")


    # --- Execute the sequence of movements ---
    for i, (x, y, z,roll,pitch,yaw) in enumerate(target_xyz_rpy_points):
        rospy.loginfo(f"\n--- Moving to Pose {i+1}: (x={x:.3f}, y={y:.3f}, z={z:.3f}) ---")


        
        target_pose = create_pose_from_rpy(x, y, z, roll, pitch, yaw) # No rotation
        move_group_arm.set_pose_target(target_pose)

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
            execute_success = move_group_arm.execute(plan_trajectory, wait=True)

            if execute_success:
                rospy.loginfo("Successfully executed trajectory to Pose %d.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final end-effector pose:\n%s", final_pose)
                time.sleep(1.0) # Short pause after successful move
            else:
                rospy.logwarn("Trajectory execution failed for Pose %d. This might be due to not reaching tolerance.", i+1)
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final approximated end-effector pose:\n%s", final_pose)
                time.sleep(1.0) # Short pause after failed move
            
            move_group_arm.stop() # Ensure no residual movement commands
            move_group_arm.clear_pose_targets() # Clear any previous targets


        # --- PICK AND PLACE LOGIC ---
        # After moving to the Pick position (index 2 in target_xyz_points)
        if i == 0: 
            rospy.loginfo("Before Pick position. Opening gripper")
            
            # Open gripper
            move_group_ee.set_named_target("ee_open")
            success_ee_open = move_group_ee.go(wait=True)
            if success_ee_open:
                rospy.loginfo("Gripper opened.")
            else:
                rospy.logwarn("Failed to open gripper.")
            move_group_ee.stop()
            time.sleep(1.0)

        elif i == 1: # At "Pick position"
            # close the gripper
            move_group_ee.set_named_target("ee_close")
            success_ee_close = move_group_ee.go(wait=True)
            if success_ee_close:
                rospy.loginfo("Gripper closed.")
            else:
                rospy.logwarn("Failed to close gripper.")
            move_group_ee.stop()
            time.sleep(3.0)        


            # Attach box to the end-effector in MoveIt's planning scene
            rospy.loginfo(f"Attaching '{box_name}' to '{eef_link}' in planning scene.")
            scene.attach_box(eef_link, box_name, touch_links=grasping_group_links)
            rospy.sleep(1.0) # Give time for attachment to register


        # After moving to the Place position (index 4 in target_xyz_points)
        elif i == 4: # At "Place position"
            rospy.loginfo("At Place position. Detaching box, then opening gripper.")
            
            # Detach box from the end-effector in MoveIt's planning scene
            rospy.loginfo(f"Detaching '{box_name}' from '{eef_link}' in planning scene.")
            scene.remove_attached_object(eef_link, name=box_name)
            rospy.sleep(1.0) # Give time for detachment to registe

            # Remove box from the world (MoveIt planning scene)
            rospy.loginfo(f"Removing '{box_name}' from planning scene.")
            scene.remove_world_object(box_name)
            rospy.sleep(1.0)
            
            # Open gripper
            move_group_ee.set_named_target("ee_open")
            success_ee_open = move_group_ee.go(wait=True)
            if success_ee_open:
                rospy.loginfo("Gripper opened.")
            else:
                rospy.logwarn("Failed to open gripper.")
            move_group_ee.stop()
            time.sleep(1.0)


            # close again the gripper
            move_group_ee.set_named_target("ee_close")
            success_ee_close = move_group_ee.go(wait=True)
            if success_ee_close:
                rospy.loginfo("Gripper closed.")
            else:
                rospy.logwarn("Failed to close gripper.")
            move_group_ee.stop()
            time.sleep(1.0)


    move_group_arm.set_named_target("pose_0") 
    success = move_group_arm.go(wait=True)
    # Check if the movement was successful
    if success:
        rospy.loginfo("Successfully moved to pose0.") 
        final_pose = move_group_arm.get_joint_value_target()
    else:
        rospy.logwarn("Approximately moved to Pose 0")
        final_pose = move_group_arm.get_joint_value_target()
        move_group_arm.stop()

    move_group_arm.stop()

    rospy.loginfo("\nFinished moving through all sequential XYZ poses and pick/place actions.")

    # --- Shutdown ---
    rospy.loginfo("Shutting down MoveIt! Commander.")
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        # Ensure Gazebo and Rviz with MoveIt are already launched
        move_arm_to_sequential_xyz_poses()
    except rospy.ROSInterruptException:
        pass


