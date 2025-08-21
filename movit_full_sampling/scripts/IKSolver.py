#!/usr/bin/env python

import rospy
import tf
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState

class IKController:
    """
    A ROS node to control a 5 DOF robotic manipulator using the MoveIt! IK service.
    
    This version of the node subscribes to a desired end-effector pose, calls the
    MoveIt! `compute_ik` service, and also continuously publishes the robot's
    current end-effector pose.
    """
    def __init__(self):
        rospy.init_node('ik_controller_node', anonymous=True)

        # 1. Initialize a publisher for the joint commands
        self.joint_pub = rospy.Publisher('/robot/joint_command', PoseStamped, queue_size=10)
        
        # 2. NEW: Publisher for the current end-effector pose
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

        # 6. Start a separate thread to continuously publish the current pose
        self.pose_publisher_thread = threading.Thread(target=self.publish_current_pose_loop)
        self.pose_publisher_thread.daemon = True # Allow the thread to exit with the main program
        self.pose_publisher_thread.start()

        rospy.loginfo("IK Controller Node initialized. Waiting for desired poses...")

    def solve_ik_with_moveit(self, pose_msg):
        """
        Calls the MoveIt! 'compute_ik' service to find a joint solution.
        
        Args:
            pose_msg (PoseStamped): The desired end-effector pose (position and orientation).
        
        Returns:
            list: A list of joint angles if a solution is found, otherwise None.
        """
        ik_request_msg = GetPositionIKRequest()
        
        ik_request_msg.ik_request.group_name = self.group_name
        
        # Set a seed joint state to help the solver.
        seed_state = RobotState()
        seed_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        seed_state.joint_state.position = [0.0] * 5
        ik_request_msg.ik_request.robot_state = seed_state

        ik_request_msg.ik_request.pose_stamped = pose_msg
        ik_request_msg.ik_request.timeout = rospy.Duration(0.1)
        ik_request_msg.ik_request.avoid_collisions = True
        
        # Set the link name for which we are calculating IK
        ik_request_msg.ik_request.ik_link_name = self.end_effector_link

        try:
            rospy.loginfo("Calling MoveIt! IK service...")
            ik_response = self.ik_service(ik_request_msg)

            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                rospy.loginfo("IK solution found successfully.")
                returnMsg = "IK service succeeded"
                return ik_response.solution.joint_state.position, returnMsg
            else:
                rospy.logwarn("IK service failed with error code: %s", ik_response.error_code.val)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None, "Service call failed"

    def pose_callback(self, pose_msg):
        """
        This function is called every time a new desired position is received.
        It passes the new position to the IK solver and ignores the orientation.
        """
        rospy.loginfo("Received new desired position.")
        
        ik_pose_msg = PoseStamped()
        ik_pose_msg.header.frame_id = self.base_link
        ik_pose_msg.header.stamp = rospy.Time.now()

        ik_pose_msg.pose.position.x = pose_msg.pose.position.x
        ik_pose_msg.pose.position.y = pose_msg.pose.position.y
        ik_pose_msg.pose.position.z = pose_msg.pose.position.z

        # Set a default, non-restrictive orientation.
        ik_pose_msg.pose.orientation.x = 0
        ik_pose_msg.pose.orientation.y = 0
        ik_pose_msg.pose.orientation.z = 0
        ik_pose_msg.pose.orientation.w = 1

        try:
            joint_angles, returnMsg = self.solve_ik_with_moveit(ik_pose_msg)

            if joint_angles:
               

                joint_cmd_msg = PoseStamped()
                joint_cmd_msg.header.stamp = rospy.Time.now()
                joint_cmd_msg.header.frame_id = returnMsg + str(joint_cmd_msg.header.seq)
                rospy.loginfo("IK solution found."+ str(joint_cmd_msg.header.seq))
                joint_cmd_msg.pose.position.x = joint_angles[0] *180/3.14
                joint_cmd_msg.pose.position.y = joint_angles[1] *180/3.14
                joint_cmd_msg.pose.position.z = joint_angles[2] *180/3.14
                joint_cmd_msg.pose.orientation.x = joint_angles[3] *180/3.14
                joint_cmd_msg.pose.orientation.y = joint_angles[4] *180/3.14
                joint_cmd_msg.pose.orientation.z = joint_angles[5] *180/3.14

                self.joint_pub.publish(joint_cmd_msg)
            else:
                rospy.logwarn("Could not find an IK solution for the given pose.")

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

