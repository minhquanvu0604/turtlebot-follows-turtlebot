#ifndef ARUCO_DETECT_HPP
#define ARUCO_DETECT_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "simple_controller.hpp"

class ArucoDetectorNode {
public:
    ArucoDetectorNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void publish_leader_tf(cv::Vec3d rvec, cv::Vec3d tvec);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cmd_vel_pub_;
    
    tf::TransformBroadcaster broadcaster_;

    float markerLength_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Mat objPoints_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    SimpleController controller_;

    // Constants
    static constexpr double DESIRED_DISTANCE = 0.7;
    static constexpr double MAX_LINEAR_SPEED = 3;
    static constexpr double MAX_ANGULAR_SPEED = 0.4;

    
};    

#endif // ARUCO_DETECT_HPP