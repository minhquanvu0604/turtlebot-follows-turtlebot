#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include "simple_controller.hpp"


class ArucoDetectorNode {
public:
    ArucoDetectorNode() 
        : it_(nh_), 
        markerLength_(0.2), 
        controller_(DESIRED_DISTANCE, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED){

        
        // Subscribe to the raw image topic
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoDetectorNode::imageCallback, this);
        // image_sub_ = it_.subscribe("/follower/camera/rgb/image_raw", 1, &ArucoDetectorNode::imageCallback, this); // topic for simulated turtlebot
              
        // Advertise the topic where the images with detected ArUco markers will be published
        image_pub_ = it_.advertise("camera/image_aruco_detected", 1);
        // cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("follower/cmd_vel", 10); // topic for simulated turtlebot
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);


        
        // Read camera parameters from tutorial_camera_params.yml
        std::string packagePath = ros::package::getPath("turtlebot_follows_turtlebot_navigation");
        std::string cameraParamsRelativePath = "/config/realsense2_camera_params.yml"; // adjust path   
        // std::string cameraParamsRelativePath = "/config/turtlebot_gazebo_camera_params.yml"; // adjust path to use simulated camera

        std::string cameraParamsFile = packagePath + cameraParamsRelativePath;

        cv::FileStorage fs(cameraParamsFile, cv::FileStorage::READ);
        if(!fs.isOpened()) {
            ROS_ERROR("Failed to open camera parameters file");
            return;
        }
        else {
        ROS_INFO("Successfully opened the camera parameters file.");
        }       
        
        if (fs["camera_matrix"].empty() || fs["distortion_coefficients"].empty()) {
            ROS_ERROR("Camera parameters are missing in the file.");
            return;
        }

        fs["camera_matrix"] >> cameraMatrix_;
        fs["distortion_coefficients"] >> distCoeffs_;

        std::cout << "cameraMatrix_ size: " << cameraMatrix_.rows << " x " << cameraMatrix_.cols << std::endl;
        std::cout << "distCoeffs_ size: " << distCoeffs_.rows << " x " << distCoeffs_.cols << std::endl;

        fs.release();

        // Set coordinate system
        objPoints_ = cv::Mat(4, 1, CV_32FC3);
        objPoints_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength_/2.f, markerLength_/2.f, 0);
        objPoints_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength_/2.f, markerLength_/2.f, 0);
        objPoints_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength_/2.f, -markerLength_/2.f, 0);
        objPoints_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength_/2.f, -markerLength_/2.f, 0);

        // ArUco dictionary and parameters
        detectorParams_ = cv::aruco::DetectorParameters::create();
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        // dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Your ArUco detection logic here:
            cv::Mat& image = cv_ptr->image;
            cv::Mat imageCopy;
            image.copyTo(imageCopy);
            
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(image, dictionary_, corners, ids, detectorParams_);
            
            ROS_INFO_STREAM("ids.size(): " << ids.size());

            // If at least one marker detected. id in this for sim robot is 24
            if (ids.size() == 1 && ids[0] == 23) {

                ROS_INFO("1");

                cv::aruco::drawDetectedMarkers(imageCopy, corners);
                
                int nMarkers = corners.size();
                std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
                
                // Calculate pose for each marker
                for (int i = 0; i < nMarkers; i++) {
                    cv::solvePnP(objPoints_, corners.at(i), cameraMatrix_, distCoeffs_, rvecs.at(i), tvecs.at(i));
                    // ROS_INFO_STREAM("rvecs: " << rvecs.at(i));
                    double distance_to_marker = tvecs.at(i)[2]; // Extracting the Z value of the translation vector
                    ROS_INFO_STREAM("DISTANCE: " << distance_to_marker);
                    
                    // Convert the distance to a string for displaying
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << distance_to_marker << " m"; // Display distance with 2 decimal points

                    // Display the distance on the image
                    cv::Point text_position(10, 30);
                    cv::putText(imageCopy, ss.str(), text_position, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                    // Control the robot                    
                    // After detecting the markers and before computing velocity commands:
                    cv::Point2f marker_center = (corners[i][0] + corners[i][2]) * 0.5; 
                    cv::Point2f image_center(image.cols / 2, image.rows / 2);

                    // Calculate displacement in pixels
                    float displacement_x = marker_center.x - image_center.x;
                    ROS_INFO_STREAM("displacement_x: " << displacement_x);
                    // ROS_INFO_STREAM("marker_center.x: " << marker_center.x);
                    // ROS_INFO_STREAM("image_center.x: " << image_center.x);
                    // ROS_INFO_STREAM("image.cols.x: " << image.cols);

                    // This calculation assumes that the field of view (FOV) of the camera is known. 
                    // FOV should be in radians.
                    // double fov_horizontal = 1.085595;  // Simulated camera fov
                    double fov_horizontal = 1.20428; // Realsense camera (69 x 42) of (H x V)
                    // double angle_to_tag = atan2(displacement_x, image_center.x) * (fov_horizontal / 2) / (image.cols / 2);
                    
                    // double pixel_ratio = displacement_x / (image.cols / 2);
                    // double angle_to_tag = pixel_ratio * (fov_horizontal / 2);
                    double angle_to_tag = atan2(displacement_x, distance_to_marker);
                    if (abs(angle_to_tag) < M_PI/5) angle_to_tag = 0;

                    ROS_INFO_STREAM("angle_to_tag" << angle_to_tag);

                    // Compute velocity commands
                    double linear_velocity, angular_velocity;
                    controller_.compute_velocity_commands(distance_to_marker, angle_to_tag, linear_velocity, angular_velocity);
                    ROS_INFO_STREAM("angular_velocity" << angular_velocity);

                    geometry_msgs::Twist cmd;
                    cmd.linear.x = linear_velocity;
                    cmd.angular.z = angular_velocity;
                    cmd_vel_pub_.publish(cmd);
                }
                
                // Draw axis for each marker
                for(unsigned int i = 0; i < ids.size(); i++) {
                    cv::aruco::drawAxis(imageCopy, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
                }
            }
            else {
                ROS_INFO("00000000000000000000");
            }

            // This next line would only make sense if you want to display the image locally.
            // It would not typically be used in a headless ROS node, but can be useful for debugging.
            cv::imshow("Aruco Detection", imageCopy);
            cv::waitKey(3);

            // Publish the image with detected ArUco markers
            image_pub_.publish(cv_ptr->toImageMsg());
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }


    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        ros::Publisher cmd_vel_pub_;
        
        float markerLength_;
        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;
        cv::Mat objPoints_;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;

        static constexpr double DESIRED_DISTANCE = 0.7;
        static constexpr double MAX_LINEAR_SPEED = 3;
        static constexpr double MAX_ANGULAR_SPEED = 0.4;

        SimpleController controller_;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector_node");
    ArucoDetectorNode node;
    ros::spin();
    return 0;
}
