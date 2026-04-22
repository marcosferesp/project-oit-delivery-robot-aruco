#include "aruco_vision/aruco_detector.hpp"

ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector_node"), got_camera_info_(false) {
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10, std::bind(&ArucoDetectorNode::infoCallback, this, std::placeholders::_1));
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArUco Brain is online and waiting for video...");
}

void ArucoDetectorNode::infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (got_camera_info_) return; 

    if (msg->k[0] == 0.0) {
        RCLCPP_WARN(this->get_logger(), "Warning: Camera sent empty calibration data. Waiting...");
        return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
    
    if (msg->d.size() > 0) {
        dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, (void*)msg->d.data()).clone();
    } else {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }
    
    got_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Calibration loaded successfully.");
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!got_camera_info_) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV translation failed!");
        return;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds);

    if (markerIds.size() > 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.10, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        for (size_t i = 0; i < markerIds.size(); i++) {
            double distance = tvecs[i][2]; 
            RCLCPP_INFO(this->get_logger(), "ArUco ID %d detected at %.2f meters away", markerIds[i], distance);
        }
    }
}