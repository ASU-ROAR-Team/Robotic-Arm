#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped

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

        # A list to hold the publisher objects
        self.publishers = []
        for topic_name in self.output_topics:
            pub = rospy.Publisher(topic_name, Float64, queue_size=10)
            self.publishers.append(pub)
            rospy.loginfo("Created publisher for topic: %s", topic_name)

        # Create a subscriber to listen for the Float64MultiArray messages
        rospy.Subscriber("/robot/joint_command", PoseStamped, self.callback)
        rospy.loginfo("Subscribed to topic: /robot/joint_command")

    def callback(self, msgInc:PoseStamped):
        """
        Callback function that processes the incoming Float64MultiArray message.
        It splits the array and publishes each element to the corresponding joint topic.
        """
        # rospy.loginfo("Received a new Float64MultiArray message.")

        # Check if the number of received data elements matches the expected number
        # if len(msg.data) != len(self.publishers):
        #     rospy.logwarn(
        #         "Received array size (%d) does not match the number of publishers (%d). Skipping.",
        #         len(msg.data), len(self.publishers)
        #     )
        #     return


        # # Iterate through the received data and the publishers
        # for i in range(len(self.publishers)- 2) :
        #     # Create a new Float64 message with the data from the array
        #     joint_value = Float64()
        #     joint_value.data = msg.data[i]/180.0 * 3.14159

        #     # Publish the new message to the corresponding topic
        #     self.publishers[i].publish(joint_value)
            # rospy.loginfo("Published data[%d]: %.2f to topic %s", i, joint_value.data, self.output_topics[i])
        # rospy.loginfo("WENT HEREE")
        msg = Float64()
        msg.data = msgInc.pose.position.x/180.0 * 3.14159
        self.publishers[0].publish(msg)
        msg = Float64()
        msg.data = msgInc.pose.position.y/180.0 * 3.14159
        self.publishers[1].publish(msg)
        msg = Float64()
        msg.data = msgInc.pose.position.z/180.0 * 3.14159
        self.publishers[2].publish(msg)
        msg = Float64()
        msg.data = msgInc.pose.orientation.x/180.0 * 3.14159
        self.publishers[3].publish(msg)
        msg = Float64()
        msg.data = msgInc.pose.orientation.y/180.0 * 3.14159
        self.publishers[4].publish(msg)
        msg = Float64()
        msg.data = msgInc.pose.orientation.z
        self.publishers[5].publish(msg)
        msg = Float64()
        msg.data = -msgInc.pose.orientation.z
        self.publishers[6].publish(msg)

        # joint_value = Float64()
        # joint_value.data = msg.data[5]
        # self.publishers[5].publish(joint_value)
        
        # joint_value = Float64()
        # joint_value.data = -msg.data[5]
        # self.publishers[6].publish(joint_value)

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
