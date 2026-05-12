/*
 * ==============================================================================
 * aruco_detector.cpp
 * Contains the vision processing logic, translating camera images to find ArUco markers and calculating their physical distance
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "aruco_detector.hpp"

/* 
 * Constructor ArucoDetectorNode
 * Prepares the node to run by initializing its network connections
 * Inputs :
 * Output :
 */
ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector_node"), got_camera_info_(false) {
    // Opens a subscriber channel on "/camera_info" with a queue size of 10 to infoCallback
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( "/camera_info", 10, std::bind(&ArucoDetectorNode::infoCallback, this, std::placeholders::_1) );
    
    // Opens a subscriber channel on "/image_raw" with a queue size of 10 to imageCallback.
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>( "/image_raw", 10, std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1) );

    // Distance Publisher
    dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/aruco_distance", 10);

    RCLCPP_INFO(this->get_logger(), "ArUco Brain is online and waiting for video...");
}

/* 
 * Function infoCallback
 * Saves the incoming physical camera calibration data into the robot's memory by updating internal matrix variables
 * Inputs : msg (sensor_msgs::msg::CameraInfo::SharedPtr) : the .yaml calibration data
 * Output :
 */
void ArucoDetectorNode::infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (got_camera_info_) return; 

    if (msg->k[0] == 0.0) {
        RCLCPP_WARN(this->get_logger(), "Warning: Camera sent empty calibration data. Waiting...");
        return;
    }

    // Converts the 9 raw ROS 2 focal numbers into a 3x3 OpenCV matrix and lock it in memory
    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
    
    if (msg->d.size() > 0) {
        dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, (void*)msg->d.data()).clone(); // Converts the raw distortion numbers into a 1x5 OpenCV matrix
    } else {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);    // If the distortion array is missing it's filled it with zeros to prevent a crash
    }
    
    got_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Calibration loaded successfully.");
}

/* 
 * Function imageCallback
 * Scans the video frame to find ArUco markers and calculates their physical distance
 * Inputs : msg (sensor_msgs::msg::Image::SharedPtr) : the raw video frame from the camera
 * Output :
 */
void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!got_camera_info_) return;

    cv_bridge::CvImagePtr cv_ptr;   // Empty pointer to hold the translated image
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // Translates the ROS 2 network frame into a Blue-Green-Red 8-bit OpenCV image matrix
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV translation failed!");
        return;
    }

    // Loads the official rules for 4x4 ArUco markers up to 50 IDs into memory
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // Scans the image, finds the squares and decodes the internal grid
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds);

    if (markerIds.size() > 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.10, camera_matrix_, dist_coeffs_, rvecs, tvecs);  // The markers are 10cm squared

        // If it scans multiple markers it prints them one after another
        for (size_t i = 0; i < markerIds.size(); i++) {
            double distance = tvecs[i][2]; 
            RCLCPP_INFO(this->get_logger(), "ArUco ID %d detected at %.2f meters away", markerIds[i], distance);
            std_msgs::msg::Float32 dist_msg;
            dist_msg.data = distance;
            dist_pub_->publish(dist_msg);
        }
    }
}