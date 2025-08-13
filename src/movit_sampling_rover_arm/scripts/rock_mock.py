#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Point

class MockHardware:
    """
    This node simulates the low-level hardware of the rover's arm.
    It provides the necessary ROS topics that `real_node.py` expects:
    1. A manual trigger for the '/camera/ready' flag.
    2. A fixed target position for '/camera/target_position'.
    3. Continuous publishing of the robot's current state on '/real_joint_states'.
    4. Subscribes to target joint commands and updates its state accordingly.
    """
    def __init__(self):
        rospy.init_node('mock_hardware_interface')
        rospy.loginfo("Mock Hardware Interface started.")

        # --- Define Joint Names (Must match URDF and MoveIt config) ---
        self.arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.hand_joint_names = ['joint_ree', 'joint_lee']
        self.all_joint_names = self.arm_joint_names + self.hand_joint_names

        # --- Internal State: Represents the robot's current encoder values ---
        self.joint_positions = {name: 0.0 for name in self.all_joint_names}
        rospy.loginfo("Initial joint states set to 0.")

        # --- Hardcoded Target Position ---
        # You can change these values to test different targets
        self.target_point = Point()
        self.target_point.x = 0.0
        self.target_point.y =  0.82
        self.target_point.z =  0.09
        rospy.loginfo(f"Mock target point set to: [x={self.target_point.x}, y={self.target_point.y}, z={self.target_point.z}]")

        # --- Publishers ---
        self.ready_pub = rospy.Publisher('/camera/ready', Bool, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher('/camera/target_position', Point, queue_size=1, latch=True)
        self.joint_state_pub = rospy.Publisher('/real_joint_states', JointState, queue_size=10)
        
        # --- Subscribers to listen for commands from real_node.py ---
        rospy.Subscriber('/rover_arm_controller/target_joints', Float64MultiArray, self.arm_command_callback)
        rospy.Subscriber('/hand_ee_controller/target_joints', Float64MultiArray, self.hand_command_callback)
    
    def _update_joints(self, joint_names, positions):
        """Helper function to update the internal state."""
        for name, pos in zip(joint_names, positions):
            if name in self.joint_positions:
                self.joint_positions[name] = pos
        # Log the received command for verification
        formatted_pos = [f"{p:.3f}" for p in positions]
        rospy.loginfo(f"Received Command. Updating {joint_names} to {formatted_pos}")

    def arm_command_callback(self, msg):
        """Updates arm joints based on received commands."""
        self._update_joints(self.arm_joint_names, msg.data)

    def hand_command_callback(self, msg):
        """Updates hand joints based on received commands."""
        self._update_joints(self.hand_joint_names, msg.data)

    def run(self):
        """Main loop for the node."""
        
        # 1. Prompt user to start the process
        while not rospy.is_shutdown():
            # Use raw_input for Python 2, input for Python 3
            choice = input("--> Set ready flag to 'true'? (y/n): ").lower()
            

            if choice == 'y':
                # Publish the camera ready and target messages once
                self.ready_pub.publish(Bool(data=True))
                self.target_pub.publish(self.target_point)
                rospy.loginfo("Published '/camera/ready=true' and '/camera/target_position'.")
                break # Exit the prompt loop
            elif choice == 'n':
                rospy.loginfo("Okay, waiting for 'y'...")
        
        if rospy.is_shutdown():
            return

        # 2. Continuously publish the robot's current state
        rate = rospy.Rate(50) # 50 Hz, a common rate for joint states
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            # Ensure the order is maintained
            msg.name = self.all_joint_names
            msg.position = [self.joint_positions[name] for name in self.all_joint_names]
            
            self.joint_state_pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        hardware = MockHardware()
        hardware.run()
    except rospy.ROSInterruptException:
        pass