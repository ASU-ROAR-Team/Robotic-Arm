#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ultralytics import YOLO

# ROS-related imports
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError


class RockDetectorNode:
    """
    This ROS node detects specific objects (rocks) using a YOLOv8 model,
    calculates their 3D position using depth data, and broadcasts this
    position as a TF frame for use by a robotic arm or other nodes.
    """

    def __init__(self):
        """Node constructor."""
        rospy.init_node('rock_detector_node')
        
        # --- Load Parameters from the ROS Parameter Server ---
        self.load_params()
        
        # --- Initialize Core Components ---
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Load the YOLOv8 model
        try:
            self.model = YOLO(self.model_path)
            rospy.loginfo("Successfully loaded YOLOv8 model from: %s", self.model_path)
        except Exception as e:
            rospy.logfatal("Failed to load YOLOv8 model: %s", e)
            rospy.signal_shutdown("Model loading failed.")
            return
            
        # Retrieve the integer class ID for our target object from the model
        self.target_class_id = self.get_class_id(self.target_class_name)
        if self.target_class_id is None:
            rospy.logfatal("Target class '%s' not found in the model's class list!", self.target_class_name)
            rospy.signal_shutdown("Invalid target class.")
            return

        # Camera intrinsics will be stored here once received
        self.camera_intrinsics = None

        # --- Subscribers using Message Filters for Synchronization ---
        # This setup ensures we process RGB, depth, and camera info messages
        # that were captured at the exact same time.
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [image_sub, depth_sub, info_sub], 
            queue_size=10
        )
        self.time_synchronizer.registerCallback(self.camera_callback)
        
        rospy.loginfo("Rock detector node initialized. Waiting for camera data...")

    def load_params(self):
        """Loads parameters from the rosparam server."""
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/color/camera_info')
        self.model_path = rospy.get_param('~model_path')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.7)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.45)
        self.target_class_name = rospy.get_param('~target_class_name', 'rock')
        self.camera_optical_frame_id = rospy.get_param('~camera_optical_frame_id', 'camera_depth_optical_frame')
        self.target_frame_base_name = rospy.get_param('~target_frame_base_name', 'target_rock')
        rospy.loginfo("Parameters loaded successfully.")
    
    def get_class_id(self, target_name):
        """Retrieves the integer ID for a given class name from the YOLO model."""
        names = self.model.names
        for class_id, class_name in names.items():
            if class_name.lower() == target_name.lower():
                rospy.loginfo("Found target class '%s' with ID: %d", target_name, class_id)
                return int(class_id)
        return None

    def camera_callback(self, rgb_msg, depth_msg, info_msg):
        """The main callback, triggered when synchronized messages are received."""
        # Store camera intrinsics if we haven't already
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': info_msg.K[0], 'fy': info_msg.K[4],
                'cx': info_msg.K[2], 'cy': info_msg.K[5]
            }
            rospy.loginfo("Camera intrinsics received and stored.")

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # The depth image can be 16UC1 (mm) or 32FC1 (meters). 'passthrough' handles both.
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CV-Bridge Error: %s", e)
            return
            
        # --- Run YOLOv8 Inference ---
        results = self.model.predict(
            source=cv_image_rgb,
            conf=self.confidence_threshold,
            iou=self.nms_threshold,
            verbose=False  # Suppress console output from YOLO
        )
        
        detections = results[0].boxes.data
        detection_id = 0
        
        # Iterate through all detected objects
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # --- Process Only the Target Object ---
            if int(class_id) == self.target_class_id:
                detection_id += 1
                
                # Calculate the 2D centroid of the bounding box
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # --- Get 3D Coordinates ---
                try:
                    # Get the depth value at the centroid
                    depth_value = cv_image_depth[center_y, center_x]

                    # If depth is 0 or NaN, it's invalid. Skip this detection.
                    if np.isnan(depth_value) or depth_value <= 0:
                        rospy.logwarn("Invalid depth value (0 or NaN) at centroid for detection %d. Skipping.", detection_id)
                        continue
                        
                    # Convert depth to meters if it's in millimeters (common for 16-bit depth images)
                    depth_in_meters = depth_value
                    if cv_image_depth.dtype == np.uint16:
                         depth_in_meters = depth_value / 1000.0

                    # Deproject the 2D pixel to a 3D point in the camera's optical frame
                    point_3d = self.deproject_pixel_to_point(center_x, center_y, depth_in_meters)
                    
                    # Broadcast this 3D point as a new TF frame
                    self.broadcast_tf_transform(point_3d, rgb_msg.header, detection_id)
                    
                except IndexError:
                    rospy.logwarn("Centroid pixel [%d, %d] is out of depth image bounds.", center_x, center_y)

    def deproject_pixel_to_point(self, u, v, depth):
        """
        Converts a 2D pixel with a depth value into a 3D point using camera intrinsics.
        (u, v) = (column, row) from top-left.
        """
        # Pinhole camera model equations
        x = (u - self.camera_intrinsics['cx']) * depth / self.camera_intrinsics['fx']
        y = (v - self.camera_intrinsics['cy']) * depth / self.camera_intrinsics['fy']
        return (x, y, depth)

    def broadcast_tf_transform(self, point_3d, header, detection_id):
        """Broadcasts the 3D point of the object's centroid as a TF transform."""
        t = TransformStamped()

        # Set the transform's header information
        t.header.stamp = header.stamp
        t.header.frame_id = self.camera_optical_frame_id  # The parent frame
        t.child_frame_id = f"{self.target_frame_base_name}_{detection_id}" # The new child frame

        # Set the translation (the 3D position of the object)
        t.transform.translation.x = point_3d[0]
        t.transform.translation.y = point_3d[1]
        t.transform.translation.z = point_3d[2]

        # Set the rotation to be identity (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        rospy.loginfo(f"Broadcasted TF for {t.child_frame_id}")


if __name__ == '__main__':

    try:
        RockDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
