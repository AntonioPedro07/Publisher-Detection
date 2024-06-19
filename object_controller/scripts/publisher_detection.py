#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ObjectDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detector', anonymous=True)

        # Publisher for distance, almost hit warning and general warnings
        self.distance_pub = rospy.Publisher('distance', Float32, queue_size=10)
        self.almost_hit_pub = rospy.Publisher('almost_hit', Bool, queue_size=10)
        self.warning_pub = rospy.Publisher('warning', String, queue_size=10)

        # Subscriber for point cloud and depth image data
        rospy.Subscriber("/royale_cam_royale_camera/point_cloud_0", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/royale_cam_royale_camera/depth_image_0", Image, self.depth_image_callback)

        # CvBridge for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Distance thresholds in meters
        self.threshold_hit = 0.5  # Distance threshold in meters for almost hitting
        self.threshold_warning = 1.0  # Distance threshold in meters for warning

    def point_cloud_callback(self, data):
        # Initialize minimum distance to a very large value
        min_distance = float('inf')

        # Iterate through the points in the point cloud
        for point in pc2.read_points(data, skip_nans=True):
            distance = point[2]  # z-coordinate represents the distance
            if distance < min_distance:
                min_distance = distance

        # Format the distance to two decimal place with the unit
        rospy.loginfo(f"Distance: {distance}")

        # Publish the formatted distance string
        self.distance_pub.publish(distance)

        # Determine if the object is almost hit based on the threshold
        almost_hit = min_distance < self.threshold_hit
        self.almost_hit_pub.publish(almost_hit)

        # Publish appropriate warnings based on the distance
        if min_distance < self.threshold_hit:
            self.warning_pub.publish("FATAL")
            rospy.loginfo("Imminent Danger!!: Object is very close!\n")
        elif min_distance < self.threshold_warning:
            self.warning_pub.publish("WARNING")
            rospy.loginfo("Warning!!: Object is close!\n")
        else:
            self.warning_pub.publish("CLEAR")
            rospy.loginfo("No danger detected\n")

    def depth_image_callback(self, data):
        try:
            # Convert the ROS Image message to a 16-bit  OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")  # Assuming depth image in 16-bit format

            # Check if the image data is all zeros 
            if np.all(cv_image == 0):
                rospy.logwarn("Depth image data is all zeros\n")
            else:
                # Normalize the depth image for visualization
                cv_image_norm = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

                # Display the normalized depth image
                cv2.imshow("Depth Image", cv_image_norm)
                cv2.waitKey(1)

        except CvBridgeError as e:
            # Log any errors that occur during the conversion
            rospy.logerr(f"CvBridgeError: {e}")

if __name__ == '__main__':
    # Create an instance of the ObjectDetector class
    od = ObjectDetector()
    try:
        # Keep the node running
        rospy.spin()
    except KeyboardInterrupt:
        # Handle shutdown gracefully
        rospy.loginfo("Shutting down object detector node.")
        cv2.destroyAllWindows()
