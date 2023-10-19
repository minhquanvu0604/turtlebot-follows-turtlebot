#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>

class ArucoDetectorNode {
public:
    ArucoDetectorNode() 
        : it_(nh_), 
        markerLength_(0.05) { // default value
        
        // Subscribe to the raw image topic
        image_sub_ = it_.subscribe("/follower/camera/rgb/image_raw", 1, &ArucoDetectorNode::imageCallback, this);
        
        // Advertise the topic where the images with detected ArUco markers will be published
        image_pub_ = it_.advertise("camera/image_aruco_detected", 1);

        // Read camera parameters from tutorial_camera_params.yml
        std::string packagePath = ros::package::getPath("turtlebot_follows_turtlebot_navigation");
        std::string cameraParamsRelativePath = "/config/turtlebot_gazebo_camera_params.yml"; // adjust path   

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
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
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


            // If at least one marker detected
            if (ids.size() > 0) {

                ROS_INFO("1");

                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                
                int nMarkers = corners.size();
                std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
                
                

                // Calculate pose for each marker
                for (int i = 0; i < nMarkers; i++) {
                    cv::solvePnP(objPoints_, corners.at(i), cameraMatrix_, distCoeffs_, rvecs.at(i), tvecs.at(i));
                    ROS_INFO_STREAM("rvecs: " << rvecs.at(i));
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
        
        float markerLength_;
        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;
        cv::Mat objPoints_;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector_node");
    ArucoDetectorNode node;
    ros::spin();
    return 0;
}
