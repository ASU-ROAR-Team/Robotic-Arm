#!/usr/bin/env python

import rospy
import tf
import threading
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint 
import math
from tf.transformations import quaternion_from_euler

class IKController:
    """
    A ROS node to control a 5 DOF robotic manipulator using the MoveIt! IK service.
    
    This version uses a single, hardcoded orientation for all IK requests.
    """
    def __init__(self):
        rospy.init_node('ik_controller_node', anonymous=True)

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.current_joint_state = [0.0] * 5
        self.lock = threading.Lock() # To safely access shared data from different threads

        # 1. Initialize a publisher for the joint commands
        self.joint_pub = rospy.Publisher('/robot/joint_command', PoseStamped, queue_size=10)
        
        # 2. Publisher for the current end-effector pose
        self.current_pose_pub = rospy.Publisher('/robot/current_pose', PoseStamped, queue_size=1)

        # 3. Initialize a TF listener to get the current pose
        self.tf_listener = tf.TransformListener()

        # 4. Wait for the MoveIt! IK service to become available
        rospy.loginfo("Waiting for MoveIt! IK service 'compute_ik'...")
        rospy.wait_for_service('compute_ik')
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)
        rospy.loginfo("MoveIt! IK service is ready.")
        
        # Define the name of the end-effector link and the group to use for IK.
        self.group_name = 'Arm_group'
        self.end_effector_link = 'link_6'
        self.base_link = 'rover' # or 'base_link' depending on your URDF

        # 5. Subscriber for the desired end-effector pose
        self.pose_sub = rospy.Subscriber('/robot/desired_pose', PoseStamped, self.pose_callback)

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.loginfo("Waiting for the first /joint_states message...")
        # Wait for the first message to ensure we have a valid seed state
        rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
        rospy.loginfo("Received first /joint_states message. IK Controller is fully initialized.")

        # 6. Start a separate thread to continuously publish the current pose
        self.pose_publisher_thread = threading.Thread(target=self.publish_current_pose_loop)
        self.pose_publisher_thread.daemon = True # Allow the thread to exit with the main program
        self.pose_publisher_thread.start()

        rospy.loginfo("IK Controller Node initialized. Waiting for desired poses...")

    def joint_states_callback(self, msg):
        '''
        NEW: This function is called every time a message is received on /joint_states.
        It updates our knowledge of the robot's current position.
        '''
        with self.lock:
            # Create a mapping from name to position for all joints in the message
            joint_map = dict(zip(msg.name, msg.position))
            # Update our stored state only for the joints we care about
            self.current_joint_state = [joint_map.get(name, 0.0) for name in self.joint_names]

    def solve_ik_with_moveit(self, pose_msg, current_joint_state):
        """
        Calls the MoveIt! 'compute_ik' service ONE TIME to find a joint solution.
        """
        ik_request_msg = GetPositionIKRequest()
        ik_request_msg.ik_request.group_name = self.group_name

        seed_state = RobotState()
        seed_state.joint_state.name = self.joint_names
        seed_state.joint_state.position = current_joint_state
        ik_request_msg.ik_request.robot_state = seed_state
        rospy.loginfo("Using seed state: %s", [round(x, 2) for x in current_joint_state])

        ik_request_msg.ik_request.pose_stamped = pose_msg
        ik_request_msg.ik_request.timeout = rospy.Duration(0.1)
        ik_request_msg.ik_request.avoid_collisions = True
        
        # Set the link name for which we are calculating IK
        ik_request_msg.ik_request.ik_link_name = self.end_effector_link

        # This is the best way to handle the 5-DOF limitation.
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_msg.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose_msg.pose.orientation
        
        # Set tolerance for each axis (in radians).
        # We constrain Pitch and Yaw (pointing) but leave Roll completely free.
        orientation_constraint.absolute_x_axis_tolerance = 6.28  # 360 degrees for Roll (free)
        orientation_constraint.absolute_y_axis_tolerance = 0.1   # ~5.7 degrees for Pitch (constrained)
        orientation_constraint.absolute_z_axis_tolerance = 0.1   # ~5.7 degrees for Yaw (constrained)
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        ik_request_msg.ik_request.constraints = constraints

        try:
            rospy.loginfo("Calling MoveIt! IK service...")
            ik_response = self.ik_service(ik_request_msg)

            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                rospy.loginfo("IK solution found successfully.")
                return ik_response.solution.joint_state.position, "IK service succeeded"
            else:
                rospy.logwarn("IK service failed with error code: %s", ik_response.error_code.val)
                return None, "IK service failed"
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None, "Service call failed"

    def pose_callback(self, pose_msg):
        """
        This function is called every time a new desired position is received.
        It combines the incoming position with a hardcoded orientation to request an IK solution.
        """
        rospy.loginfo("Received new desired position.")

        # Hardcoded orientation in Euler angles (roll, pitch, yaw)
        roll_rad = 3.08
        pitch_rad = 0.017
        yaw_rad = 1.578

        # Convert the Euler angles (roll, pitch, yaw) to a quaternion
        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        
        # Build the final PoseStamped message for the IK solver
        ik_pose_msg = PoseStamped()
        ik_pose_msg.header.frame_id = self.base_link
        ik_pose_msg.header.stamp = rospy.Time.now()
        # Use the position from the incoming message
        ik_pose_msg.pose.position = pose_msg.pose.position
        # Use our hardcoded orientation
        ik_pose_msg.pose.orientation.x = quaternion[0]
        ik_pose_msg.pose.orientation.y = quaternion[1]
        ik_pose_msg.pose.orientation.z = quaternion[2]
        ik_pose_msg.pose.orientation.w = quaternion[3]

        try:
            with self.lock:
                latest_joint_state = list(self.current_joint_state)
            
            # Call the solver just once with our constructed pose
            joint_angles, returnMsg = self.solve_ik_with_moveit(ik_pose_msg, latest_joint_state)

            if joint_angles:
                joint_cmd_msg = PoseStamped()
                joint_cmd_msg.header.stamp = rospy.Time.now()
                joint_cmd_msg.header.frame_id = returnMsg + str(joint_cmd_msg.header.seq)
                rospy.loginfo("IK solution found."+ str(joint_cmd_msg.header.seq))
                
                # Unpack the 5 joint angles from the IK solution
                joint_cmd_msg.pose.position.x = joint_angles[0] * 180/3.14
                joint_cmd_msg.pose.position.y = joint_angles[1] * 180/3.14
                joint_cmd_msg.pose.position.z = joint_angles[2] * 180/3.14
                joint_cmd_msg.pose.orientation.x = joint_angles[3] * 180/3.14
                joint_cmd_msg.pose.orientation.y = joint_angles[4] * 180/3.14
                joint_cmd_msg.pose.orientation.z = 0.0
                self.joint_pub.publish(joint_cmd_msg)
            else:
                rospy.logwarn("Could not find an IK solution for the given pose and fixed orientation.")

        except Exception as e:
            rospy.logerr("An error occurred during IK calculation: %s", str(e))

    def publish_current_pose_loop(self):
        """
        Periodically publishes the current end-effector pose.
        This runs in a separate thread.
        """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            try:
                # Wait for the transform to become available
                self.tf_listener.waitForTransform(self.base_link, self.end_effector_link, rospy.Time(0), rospy.Duration(1.0))
                
                # Get the current transform
                (trans, rot) = self.tf_listener.lookupTransform(self.base_link, self.end_effector_link, rospy.Time(0))
                
                # Create and populate a PoseStamped message
                current_pose_msg = PoseStamped()
                current_pose_msg.header.stamp = rospy.Time.now()
                current_pose_msg.header.frame_id = self.base_link
                
                current_pose_msg.pose.position.x = trans[0]
                current_pose_msg.pose.position.y = trans[1]
                current_pose_msg.pose.position.z = trans[2]
                
                current_pose_msg.pose.orientation.x = rot[0]
                current_pose_msg.pose.orientation.y = rot[1]
                current_pose_msg.pose.orientation.z = rot[2]
                current_pose_msg.pose.orientation.w = rot[3]
                
                # Publish the message
                self.current_pose_pub.publish(current_pose_msg)
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("Could not get transform to publish current pose: %s", str(e))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        ik_node = IKController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass