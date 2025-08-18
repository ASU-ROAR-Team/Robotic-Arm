#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
#import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Point
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult

class CombinedArmNode:
    """
    A combined ROS node that provides both a low-level interface (action servers for
    trajectory execution) and a high-level task logic (waiting for a camera flag
    and performing a pick-and-place task).
    
    This version uses a callback-based approach for the camera topics, storing
    the data in class variables for later use.
    """
    def __init__(self):
        rospy.init_node('real_arm_node', anonymous=True)
        rospy.loginfo("Combined Arm Node Initialized with feedback control and task logic.")
        
        # --- Robot State Variables ---
        self.current_joint_positions = {}
        self.waypoint_tolerance = 0.05 # 5% tolerance for reaching a waypoint
        self.arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'] # Arm joints
        self.hand_joint_names = ['joint_ree', 'joint_lee'] # Hand joints
        
        # --- Camera State Variables (new) ---
        self.rock_ready = False
        self.sand_ready = False
        self.rock_target_point = None
        self.sand_target_point = None

        # -- Task COMPLETION Flags (mission success flags) --
        self.rock_mission_complete = False # Set True by /rock/box_status, locks out the mission
        self.sand_mission_complete = False # Set True by /sand/box_status, locks out the mission
        
        # -- Task IN_PROGRESS Flags (prevents interruption) --
        self.rock_in_progress = False
        self.sand_in_progress = False

        # --- Joint Publishers and Subscribers ---
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.real_joint_state_sub = rospy.Subscriber('/real_joint_states', JointState, self.real_joint_states_callback)
        self.arm_command_pub = rospy.Publisher('/rover_arm_controller/target_joints', Float64MultiArray, queue_size=10)
        self.hand_command_pub = rospy.Publisher('/hand_ee_controller/target_joints', Float64MultiArray, queue_size=10)
        
        # === Mission Control Subscribers ===
        rospy.Subscriber('/sand/ready', Bool, self._sand_ready_callback)
        rospy.Subscriber('/rock/ready', Bool, self._rock_ready_callback)
        rospy.Subscriber('/camera/rock_position', Point, self._rock_position_callback)
        rospy.Subscriber('/camera/sand_position', Point, self._sand_position_callback)
        rospy.Subscriber('/rock/box_status', Bool, self._rock_mission_complete_callback)
        rospy.Subscriber('/sand/box_status', Bool, self._sand_mission_complete_callback)


    # --- New Camera Callback Methods ---
    def _rock_ready_callback(self, msg):
        """
        Callback to set the rock_ready flag.
        """
        self.rock_ready = msg.data
        if self.rock_ready:
            rospy.loginfo("Received 'rock_ready' flag = True.")

    # --- New Camera Callback Methods ---
    def _sand_ready_callback(self, msg):
        """
        Callback to set the sand_ready flag.
        """
        self.sand_ready = msg.data
        if self.sand_ready:
            rospy.loginfo("Received 'sand_ready' flag = True.")
                        
    def _rock_position_callback(self, msg):
        """
        Callback to store the latest target position.
        """
        self.rock_target_point = [msg.x, msg.y, msg.z]
        rospy.loginfo("Received target position via callback: [%.3f, %.3f, %.3f].",
                      self.rock_target_point[0], self.rock_target_point[1], self.rock_target_point[2])

    def _sand_position_callback(self, msg):
        """
        Callback to store the latest target position.
        """
        self.sand_target_point = [msg.x, msg.y, msg.z]
        rospy.loginfo("Received target position via callback: [%.3f, %.3f, %.3f].",
                      self.sand_target_point[0], self.sand_target_point[1], self.sand_target_point[2])    
        
    def _rock_mission_complete_callback(self, msg):
        """
        Callback to handle box status updates.
        """
        if msg.data and not self.rock_mission_complete:
             rospy.loginfo("ROCK mission completion flag received from /rock/box_status. Rock task will be permanently disabled after current run.")
        self.rock_mission_complete = msg.data

    def _sand_mission_complete_callback(self, msg):
        """
        Callback to handle box status updates.
        """
        # Set the sampling_ready flag based on the box status
        if msg.data and not self.sand_mission_complete:
             rospy.loginfo("SAND mission completion flag received from /rock/box_status. Rock task will be permanently disabled after current run.")
        self.sand_mission_complete = msg.data


    # --- Existing Callback and Trajectory Execution Methods ---
    def real_joint_states_callback(self, msg):
        """
        Receives encoder values, stores them for feedback control, and re-publishes
        to the standard /joint_states topic.
        """
        msg.header.stamp = rospy.Time.now()
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos
        self.joint_state_pub.publish(msg)
    
    def execute_trajectory(self, publisher, full_trajectory):
        """
        Executes a trajectory manually using a closed-loop feedback mechanism.
        This function has no dependency on Action Servers.

        Args:
            publisher: The ROS topic publisher to send joint commands to.
            trajectory: The trajectory_msgs/JointTrajectory to execute.
            joint_names: An ordered list of joint names corresponding to the trajectory.

        Returns:
            True on successful execution, False on failure oF LASTPOINT.
        """
        trajectory= full_trajectory.joint_trajectory
        joint_names = trajectory.joint_names
        rospy.loginfo(f"Starting manual execution for joints: {joint_names}")
        
        # Check if the trajectory is empty
        if not trajectory.points:
            rospy.logerr("Received an empty trajectory. Aborting execution.")
            return False

        # Create a message to reuse for publishing
        target_joints_msg = Float64MultiArray()
        num_waypoints = len(trajectory.points)
        # Iterate through each point (waypoint) in the trajectory
        for i, point in enumerate(trajectory.points):
  
            rospy.loginfo(f"--> Moving to Waypoint {i+1}/{num_waypoints}")
            is_final_waypoint = (i == num_waypoints - 1)
            
            # 1. SEND THE COMMAND for the current waypoint
            target_joints_msg.data = point.positions
            publisher.publish(target_joints_msg)

            # 2. WAIT FOR FEEDBACK until the robot reaches the waypoint
            start_time = rospy.Time.now()
            timeout_duration = rospy.Duration(1.0)  
            waypoint_reached = False # Flag to track success

            while (rospy.Time.now() - start_time) < timeout_duration:
                # Safety check: wait until we have at least one feedback message
                if not self.current_joint_positions:
                    rospy.sleep(0.01)
                    continue

                # Check if all joints have reached their target positions
                all_joints_reached = True
                for name, target_pos in zip(joint_names, point.positions):
                    current_pos = self.current_joint_positions.get(name, None)
                    if current_pos is None or abs(target_pos - current_pos) > self.waypoint_tolerance:
                        all_joints_reached = False
                        break  # At least one joint is not there yet, stop checking

                if all_joints_reached:
                    rospy.loginfo(f"    Waypoint {i+1} reached.")
                    waypoint_reached = True
                    break # Success! Exit the waiting (while) loop
                
                rospy.sleep(0.01) # Prevent busy-waiting

            else: # This 'else' clause triggers ONLY if the 'while' loop times out
                rospy.logwarn(f"Timeout waiting for waypoint {i+1}.")
                return False # Signal that the execution failed
            
                        # After the 'while' loop (either by break or timeout), check the result
            if not waypoint_reached:
                rospy.logwarn(f"Timeout waiting for waypoint {i+1}.")
                # ==================== NEW LOGIC ====================
                if is_final_waypoint:
                    rospy.logerr("FINAL waypoint failed to reach. Execution FAILED.")
                    return False # Strict failure for the final point
                else:
                    rospy.logwarn("...this is an intermediate waypoint, SKIPPING and continuing.")
                    # We simply continue to the next iteration of the 'for' loop

        # After the 'for' loop finishes, we must do a final check.
        # It's possible the last waypoint was the one that timed out.
        # The 'waypoint_reached' flag will be False in that case.
        if waypoint_reached:
            rospy.loginfo("Trajectory execution completed successfully (final waypoint reached).")
            return True
        else:
            # This case handles when the loop ends because the last waypoint failed.
            rospy.logerr("Trajectory execution FAILED because the final waypoint was not reached.")
            return False

    def run_rock_sequence(self, arm_group, gripper_group):
        """
        This is the high-level task logic, modified to use the class variables.
        It waits for a camera signal, uses the stored target of the rock, and performs the
        Task sequence.
        """
        rospy.loginfo("========== STARTING ROCK SEQUENCE ==========")
        self.rock_in_progress = True # Lock out other tasks
        hand_publisher = self.hand_command_pub
        arm_publisher = self.arm_command_pub
        move_group_arm = arm_group
        move_group_ee = gripper_group


        # Use the stored target point for the rock task
        rock_target_xyz = self.rock_target_point
        rospy.loginfo("Using received target position: [%.3f, %.3f, %.3f].",
                      rock_target_xyz[0], rock_target_xyz[1], rock_target_xyz[2])
        
        # Ensure arm returns to a known 'pose0' at start of script
        rospy.loginfo("\nAttempting to move to pose0 (home/start position).")
        move_group_arm.set_named_target("pose0") # Make sure "pose0" is defined in your SRDF or MoveIt config
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        execution_succeeded = self.execute_trajectory(arm_publisher, plan_trajectory)
        if execution_succeeded:
            rospy.loginfo("Successfully moved to pose0.")
        else:
            rospy.logwarn("Approximately moved to Pose0 or failed to reach. Continuing anyway.")
            move_group_arm.stop() # Stop any residual motion


        # move to pre_pick pose before each target
        rospy.loginfo("\nAttempting to move to pre pick position.")
        move_group_arm.set_named_target("pre_pick") # Make sure "pre_pick" is defined in your SRDF or MoveIt config
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        execution_succeeded = self.execute_trajectory(arm_publisher, plan_trajectory)
        if execution_succeeded:
            rospy.loginfo("Successfully moved to pre pose.")
        else:
            rospy.logwarn("Approximately moved to Pre pose or failed to reach. Continuing anyway.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper before moving to a new target to grasp the rocks
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") # Make sure "ee_open" is defined for your gripper group
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_open = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep

        # Move to the camera-provided target
        x, y, z = rock_target_xyz
        rospy.loginfo(f"\n--- Moving to Target: (x={x:.3f}, y={y:.3f}, z={z:.3f}) ---")
        move_group_arm.set_position_target([x, y, z])
        move_group_arm.set_planning_time(5.0)
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
            # Print the waypoints for visualization
            print("-" * 50)
            print("PLANNED TRAJECTORY WAYPOINTS (Joint Angles in Radians)")
            print("-" * 50)
            for i, point in enumerate(plan_trajectory.joint_trajectory.points):
                # Format each position to 3 decimal places for readability
                formatted_positions = [f"{pos:.3f}" for pos in point.positions]
                print(f"Point {i}: {formatted_positions}")
            print("-" * 50)

            # Ask for user confirmation before execution
            # Use raw_input for Python 2, input for Python 3
            confirm = input("--> Execute this plan? (y/n): ").lower()
            if confirm == 'y':
                rospy.loginfo("User confirmed. Executing trajectory...")
                execution_succeeded = self.execute_trajectory(arm_publisher, plan_trajectory)
                # ... the rest of the success/fail logic ...
            else:
                rospy.loginfo("Execution aborted by user.")
                execution_succeeded = False

            if execution_succeeded:
                rospy.loginfo("Successfully executed trajectory")
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final end-effector pose (position and ACHIEVED orientation):\n%s", final_pose)
                rospy.sleep(1.0)
            else:
                rospy.logwarn("Trajectory execution failed for Target. This might be due to not reaching tolerance.")
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final approximated end-effector pose:\n%s", final_pose)
                rospy.sleep(1.0)
            # ...and so on...
                         
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()

        
        # Close gripper after reaching a target (e.g., after "grasping" something at the location)
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") # Make sure "ee_close" is defined for your gripper group
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_close = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep


        rospy.loginfo("\nAttempting to move to place rock pose.")
        move_group_arm.set_named_target("place_rock") # Make sure "place_rock" is defined in your SRDF 
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        success_place = self.execute_trajectory(arm_publisher, plan_trajectory)
        if success_place:
            rospy.loginfo("Successfully moved to place rock.")
        else:
            rospy.logwarn("Approximately moved to place rock.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper to drop the rock
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") 
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan()  
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))

        success_ee_open = self.execute_trajectory(hand_publisher, plan_trajectory)
        # Check if the gripper opened successfully
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

        # Close gripper after dropping the rock
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") 
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan()  
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_close = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

        # Final return to 'pose0' for safety/reset
        rospy.loginfo("\nAttempting final return to pose0.")
        move_group_arm.set_named_target("pose0")
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        success_final_home = self.execute_trajectory(arm_publisher, plan_trajectory)
        if not success_final_home:
            rospy.logwarn("Failed to return to pose0.")
    
        # --- Final cleanup for this sequence run ---
        self.rock_position = None # Clear data to wait for next trigger
        self.rock_ready = False   # Reset ready flag
        self.rock_in_progress = False # Unlock for other tasks
        rospy.loginfo("========== ROCK SEQUENCE FINISHED ==========\n")

    def run_sand_sequence(self, arm_group, gripper_group):
        """
        This is the high-level task logic, modified to use the class variables.
        It waits for a camera signal, uses the stored target, and performs the
        Task sequence.
        """
        rospy.loginfo("========== STARTING SAND SEQUENCE ==========")
        self.sand_in_progress = True # Lock out other tasks
        hand_publisher = self.hand_command_pub
        arm_publisher = self.arm_command_pub
        move_group_arm = arm_group
        move_group_ee = gripper_group

        # Use the stored target point for the task
        sand_target_xyz = self.sand_target_point
        rospy.loginfo("Using received target position: [%.3f, %.3f, %.3f].",
                      sand_target_xyz[0], sand_target_xyz[1], sand_target_xyz[2])
        
        # Ensure arm returns to a known 'pose0' at start of script
        rospy.loginfo("\nAttempting to move to pose0 (home/start position).")
        move_group_arm.set_named_target("pose0") # Make sure "pose0" is defined in your SRDF or MoveIt config
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        execution_succeeded = self.execute_trajectory(arm_publisher, plan_trajectory)
        if execution_succeeded:
            rospy.loginfo("Successfully moved to pose0.")
        else:
            rospy.logwarn("Approximately moved to Pose0 or failed to reach. Continuing anyway.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper before moving to a new target to grasp the sand
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") # Make sure "ee_open" is defined for your gripper group
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan()  
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_open = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep

        # Move to the camera-provided sand target
        x, y, z = sand_target_xyz
        rospy.loginfo(f"\n--- Moving to Target: (x={x:.3f}, y={y:.3f}, z={z:.3f}) ---")
        move_group_arm.set_position_target([x, y, z])
        move_group_arm.set_planning_time(5.0)
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan()         
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
            # Print the waypoints for visualization
            print("-" * 50)
            print("PLANNED TRAJECTORY WAYPOINTS (Joint Angles in Radians)")
            print("-" * 50)
            for i, point in enumerate(plan_trajectory.joint_trajectory.points):
                # Format each position to 3 decimal places for readability
                formatted_positions = [f"{pos:.3f}" for pos in point.positions]
                print(f"Point {i}: {formatted_positions}")
            print("-" * 50)

            # Ask for user confirmation before execution
            confirm = input("--> Execute this plan? (y/n): ").lower()
            if confirm == 'y':
                rospy.loginfo("User confirmed. Executing trajectory...")
                execution_succeeded = self.execute_trajectory(arm_publisher, plan_trajectory)
                # ... the rest of the success/fail logic ...
            else:
                rospy.loginfo("Execution aborted by user.")
                execution_succeeded = False

            if execution_succeeded:
                rospy.loginfo("Successfully executed trajectory")
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final end-effector pose (position and ACHIEVED orientation):\n%s", final_pose)
                rospy.sleep(1.0)
            else:
                rospy.logwarn("Trajectory execution failed for Target. This might be due to not reaching tolerance.")
                final_pose = move_group_arm.get_current_pose().pose
                rospy.loginfo("Final approximated end-effector pose:\n%s", final_pose)
                rospy.sleep(1.0)
                
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()

        
        # Close gripper after reaching a target (e.g., after "grasping" something at the location)
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") # Make sure "ee_close" is defined for your gripper group
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan()                
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_close = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) # ROS-aware sleep


        rospy.loginfo("\nAttempting to move to place sand pose.")
        move_group_arm.set_named_target("place_sand") # Make sure "place_sand" is defined in your SRDF 
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        success_place = self.execute_trajectory(arm_publisher, plan_trajectory)
        if success_place:
            rospy.loginfo("Successfully moved to place sand.")
        else:
            rospy.logwarn("Approximately moved to place sand.")
            move_group_arm.stop() # Stop any residual motion

        # Open gripper to drop the rock
        rospy.loginfo("Opening gripper.")
        move_group_ee.set_named_target("ee_open") 
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))

        success_ee_open = self.execute_trajectory(hand_publisher, plan_trajectory)
        # Check if the gripper opened successfully
        if success_ee_open:
            rospy.loginfo("Gripper opened.")
        else:
            rospy.logwarn("Failed to open gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

        # Close gripper after dropping the sand
        rospy.loginfo("Closing gripper.")
        move_group_ee.set_named_target("ee_close") 
        success_plan ,plan_trajectory , _ , _ = move_group_ee.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_ee.stop()
            move_group_ee.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
               
        success_ee_close = self.execute_trajectory(hand_publisher, plan_trajectory)
        if success_ee_close:
            rospy.loginfo("Gripper closed.")
        else:
            rospy.logwarn("Failed to close gripper.")
        move_group_ee.stop()
        rospy.sleep(1.0) 

        # Final return to 'pose0' for safety/reset
        rospy.loginfo("\nAttempting final return to pose0.")
        move_group_arm.set_named_target("pose0")
        success_plan ,plan_trajectory , _ , _ = move_group_arm.plan() 
        if not success_plan or plan_trajectory is None or len(plan_trajectory.joint_trajectory.points) == 0:
            rospy.logwarn("Planning failed or returned an empty trajectory. Skipping execution.")
            move_group_arm.stop()
            move_group_arm.clear_pose_targets()
        else:
            rospy.loginfo("Planning successful. Trajectory contains %d waypoints.", len(plan_trajectory.joint_trajectory.points))
            
        success_final_home = self.execute_trajectory(arm_publisher, plan_trajectory)
        if not success_final_home:
            rospy.logwarn("Failed to return to pose0.")
    
        self.sand_position = None # Clear data
        self.sand_ready = False   # Reset ready flag
        self.sand_in_progress = False # Unlock
        rospy.loginfo("========== SAND SEQUENCE FINISHED ==========\n")


    
    # === MAIN CONTROL LOOP ===
    def run(self):
        # === MoveIt! Objects (Initialized once) ===
        rospy.loginfo("Initializing MoveIt! Commander...")
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group_arm = moveit_commander.MoveGroupCommander("Arm_group")
        move_group_ee = moveit_commander.MoveGroupCommander("EE_group")
        move_group_arm.set_end_effector_link("link_5")
        rospy.sleep(2.0)  # Allow time for MoveIt! to initialize
        rospy.loginfo("MoveIt! is ready.")

        rate = rospy.Rate(1) # Check for new tasks once per second
        rospy.loginfo("Mission Controller is running. Waiting for tasks.")

        while not rospy.is_shutdown():
            if self.rock_mission_complete and self.sand_mission_complete:
                rospy.loginfo("All missions are complete. Shutting down.")
                break
            
            # Prevent tasks from interrupting each other
            if self.rock_in_progress or self.sand_in_progress:
                rate.sleep()
                continue
                
            # --- Task Dispatcher Logic ---
            # Condition: Rock task is ready, position data received, and mission isn't permanently done
            if self.rock_ready and self.rock_position is not None and not self.rock_mission_complete:
                self.run_rock_sequence(move_group_arm, move_group_ee)
            
            # Condition: Sand task is ready, position data received, and mission isn't permanently done
            elif self.sand_ready and self.sand_position is not None and not self.sand_mission_complete:
                self.run_sand_sequence(move_group_arm, move_group_ee)

            rate.sleep()
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        node = CombinedArmNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down node.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", str(e))
    finally:
        pass