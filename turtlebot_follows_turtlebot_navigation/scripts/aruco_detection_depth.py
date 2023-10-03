#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco

# from cv2 import aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose

class ArUcoDepthDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_subscriber = rospy.Subscriber("follower/camera/rgb/image_raw", Image, self.rgb_callback)
        self.depth_subscriber = rospy.Subscriber("follower/camera/depth/image_raw", Image, self.depth_callback)

        self.camera_info = rospy.wait_for_message("follower/camera/rgb/camera_info", CameraInfo)
        self._dist_coeffs = np.zeros((4,1))
        self._camera_matrix = np.array(self.camera_info.K).reshape(3, 3)
        # Publisher
        self.pose_publisher = rospy.Publisher("/aruco_depth_pose", Pose, queue_size=10)


        self.latest_depth_image = None

    def rgb_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("RGB Image", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        # parameters = aruco.DetectorParameters()

        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            print("Found ArUco marker!")
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, self._camera_matrix , self._dist_coeffs)

            # Using depth information to refine or validate the pose estimation
            marker_center = np.mean(corners[0][0], axis=0).astype(int)
            depth_at_marker = self.latest_depth_image[marker_center[1], marker_center[0]]

            # Use depth_at_marker to refine tvec or validate its accuracy.

            pose = Pose()
            # Fill the pose data based on the refined or validated tvec and rvec

            self.pose_publisher.publish(pose)
        else: 
            print("No ArUco marker found!")

    def depth_callback(self, data):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('aruco_depth_detector_node', anonymous=True)
    detector = ArUcoDepthDetector()
    rospy.spin()
