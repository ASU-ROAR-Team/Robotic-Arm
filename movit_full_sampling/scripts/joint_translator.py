#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool ,Float64MultiArray
from geometry_msgs.msg import PoseStamped
from roar_msgs.msg import ArmControl # type: ignore
import math
class JointCommandSplitter:
    """
    A ROS node that subscribes to a single Float64MultiArray message
    and publishes each element to a separate Float64 topic.
    """
    def __init__(self):
        """
        Initializes the node, creates the publishers, and sets up the subscriber.
        """
        # Initialize the ROS node with a unique name
        rospy.init_node('joint_command_splitter', anonymous=True)
        rospy.loginfo("Node 'joint_command_splitter' started.")

        # Define the names of the output topics for your arm joints
        self.output_topics = [
            "/joint_1_position_controller/command",
            "/joint_2_position_controller/command",
            "/joint_3_position_controller/command",
            "/joint_4_position_controller/command",
            "/joint_5_position_controller/command",
            "/joint_ree_position_controller/command",
            "/joint_lee_position_controller/command"
        ]

        self.last_joint_rad = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'end_effector': 0.0
        }

        self.regolith_flag = False
        self.rock_flag = False
        
        # A list to hold the publisher objects
        self.publishers = []
        for topic_name in self.output_topics:
            pub = rospy.Publisher(topic_name, Float64, queue_size=10)
            self.publishers.append(pub)
            rospy.loginfo("Created publisher for topic: %s", topic_name)

        # --- CHANGE 2: Create a new publisher for your custom ArmControl message ---
        self.arm_control_pub = rospy.Publisher('/arm_control_values', ArmControl, queue_size=10)
        rospy.loginfo("Created publisher for topic: /arm_control_values")

        # Create a subscriber to listen for the Float64MultiArray messages
        rospy.Subscriber("/robot/joint_command", PoseStamped, self.joint_callback)
        rospy.loginfo("Subscribed to topic: /robot/joint_command")

        rospy.Subscriber("/robot/rock_storage", Bool, self.rock_callback)
        rospy.loginfo("Subscribed to topic: /robot/rock_storage")

        rospy.Subscriber("/robot/regolith_storage", Bool, self.regolith_callback)
        rospy.loginfo("Subscribed to topic: /robot/regolith_storage")

    def rock_callback(self, msg:Bool):
        """
        Callback function for the rock storage topic.
        Updates the rock_flag based on the received message.
        """
        self.rock_flag = msg.data
        rospy.loginfo("Rock storage flag updated: %s", self.rock_flag)
        self.publish_state()

    def regolith_callback(self, msg:Bool):
        """
        Callback function for the regolith storage topic.
        Updates the regolith_flag based on the received message.
        """
        self.regolith_flag = msg.data
        rospy.loginfo("Regolith storage flag updated: %s", self.regolith_flag)
        self.publish_state()

    def publish_state(self):
        arm_control_msg = ArmControl()
        arm_control_msg.header.stamp = rospy.Time.now()
        arm_control_msg.header.frame_id = "base_link"
        arm_control_msg.joint1 = self.last_joint_rad['joint1']
        arm_control_msg.joint2 = self.last_joint_rad['joint2']
        arm_control_msg.joint3 = self.last_joint_rad['joint3']
        arm_control_msg.joint4 = self.last_joint_rad['joint4']
        arm_control_msg.joint5 = self.last_joint_rad['joint5']
        arm_control_msg.end_effector = self.last_joint_rad['end_effector']      
        arm_control_msg.regolith = self.regolith_flag
        arm_control_msg.rock = self.rock_flag
        self.arm_control_pub.publish(arm_control_msg)

    def joint_callback(self, msg_in:PoseStamped):
        """
        Callback function that processes the incoming Float64MultiArray message.
        It splits the array and publishes each element to the corresponding joint topic.
        """

        # update the joint angles in the ArmControl message
        self.last_joint_rad['joint1'] = msg_in.pose.position.x / 180.0 * math.pi
        self.last_joint_rad['joint2'] = msg_in.pose.position.y / 180.0 * math.pi
        self.last_joint_rad['joint3'] = msg_in.pose.position.z / 180.0 * math.pi
        self.last_joint_rad['joint4'] = msg_in.pose.orientation.x / 180.0 * math.pi
        self.last_joint_rad['joint5'] = msg_in.pose.orientation.y / 180.0 * math.pi
        self.last_joint_rad['end_effector'] = msg_in.pose.orientation.z

        self.publishers[0].publish(Float64(self.last_joint_rad['joint1']))
        self.publishers[1].publish(Float64(self.last_joint_rad['joint2']))
        self.publishers[2].publish(Float64(self.last_joint_rad['joint3']))
        self.publishers[3].publish(Float64(self.last_joint_rad['joint4']))
        self.publishers[4].publish(Float64(self.last_joint_rad['joint5']))
        self.publishers[5].publish(Float64(self.last_joint_rad['end_effector']))
        self.publish_state()


def main():
    """
    Main function to run the ROS node.
    """
    try:
        node = JointCommandSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shut down.")

if __name__ == '__main__':
    main()
