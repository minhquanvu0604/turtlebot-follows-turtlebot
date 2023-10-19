#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class ArUcoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("/turtlebot/image_raw", Image, self.image_callback)
        self.pose_publisher = rospy.Publisher("/aruco_pose", Pose, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        # Detect the markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Here, I'm assuming one marker for simplicity. In reality, you'd want to loop over detected markers.
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, camera_matrix, dist_coeffs)
            
            # Convert rvec to a rotation matrix
            R_mtx, jac = cv2.Rodrigues(rvec)

            # Convert rotation matrix to euler angles
            pitch, yaw, roll = self.rotation_matrix_to_euler_angles(R_mtx)

            # Publish the pose
            pose = Pose()
            pose.position.x = tvec[0][0][0]
            pose.position.y = tvec[0][0][1]
            pose.position.z = tvec[0][0][2]
            pose.orientation.x = roll
            pose.orientation.y = pitch
            pose.orientation.z = yaw

            self.pose_publisher.publish(pose)

    @staticmethod
    def rotation_matrix_to_euler_angles(R):
        """
        Convert a rotation matrix to Euler angles using the ZYX convention.
        Returns the angles in radians.
        """
        # Ensure proper rotation matrix
        assert (R.shape == (3, 3)), "Invalid rotation matrix dimensions."

        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
        

if __name__ == '__main__':
    rospy.init_node('aruco_detector_node', anonymous=True)
    detector = ArUcoDetector()
    rospy.spin()
