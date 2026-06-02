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
    // dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("/aruco_distance", 10);
    coord_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/aruco_coordinates", 10);
    id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco_id", 10);

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

    double distance = 0.0;
    float center_x = 0.0, center_y = 0.0, dx = 0.0, dy = 0.0, marker_angle = 0.0;
    float c0x = 0.0, c1x = 0.0, c2x = 0.0, c3x = 0.0;
    float c0y = 0.0, c1y = 0.0, c2y = 0.0, c3y = 0.0;
    float top_mid_x = 0.0, top_mid_y = 0.0, bot_mid_x = 0.0, bot_mid_y = 0.0;

    geometry_msgs::msg::Point coord_msg;
    std_msgs::msg::Int32 id_msg;

    if (markerIds.size() > 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.10, camera_matrix_, dist_coeffs_, rvecs, tvecs);  // The markers are 10cm squared

        // If it scans multiple markers it prints them one after another
        for (size_t i = 0; i < markerIds.size(); i++) {
            distance = tvecs[i][2];

            // Extract the raw corners for exhaustive telemetry
            c0x = markerCorners[i][0].x, c0y = markerCorners[i][0].y;
            c1x = markerCorners[i][1].x, c1y = markerCorners[i][1].y;
            c2x = markerCorners[i][2].x, c2y = markerCorners[i][2].y;
            c3x = markerCorners[i][3].x, c3y = markerCorners[i][3].y;

            center_x = (c0x + c1x + c2x + c3x) / 4.0;
            center_y = (c0y + c1y + c2y + c3y) / 4.0;
            coord_msg.x = center_x;
            coord_msg.y = center_y;

            // 1. Find the exact midpoints
            top_mid_x = (c0x + c1x) / 2.0;
            top_mid_y = (c0y + c1y) / 2.0;
            bot_mid_x = (c2x + c3x) / 2.0;
            bot_mid_y = (c2y + c3y) / 2.0;

            // 2. Calculate the 2D vector
            dx = top_mid_x - bot_mid_x;
            dy = top_mid_y - bot_mid_y;

            // 3. True 2D rotation
            marker_angle = atan2(dx, -dy);
            coord_msg.z = marker_angle;

            coord_pub_->publish(coord_msg);
            id_msg.data = markerIds[i];
            id_pub_->publish(id_msg);

            // =========================================================================
            // EXHAUSTIVE DETECTOR LOGGING
            // =========================================================================
            RCLCPP_INFO(this->get_logger(), "==== DETECTOR CALCS [ID: %d] ====", markerIds[i]);
            RCLCPP_INFO(this->get_logger(), "Corners -> TL: %.1f,%.1f | TR: %.1f,%.1f | BR: %.1f,%.1f | BL: %.1f,%.1f", c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y);
            RCLCPP_INFO(this->get_logger(), "Midpts  -> TOP: %.1f,%.1f | BOT: %.1f,%.1f", top_mid_x, top_mid_y, bot_mid_x, bot_mid_y);
            RCLCPP_INFO(this->get_logger(), "Vector  -> dx: %.1f | dy: %.1f", dx, dy);
            RCLCPP_INFO(this->get_logger(), "Output  -> Dist: %.2fm | Cntr: %.1f,%.1f | Angle: %.4f rad", distance, center_x, center_y, marker_angle);
            // RCLCPP_INFO(this->get_logger(), "ArUco ID %d detected at %.2f meters away. Center is at X=%.1f, Y=%.1f. ", markerIds[i], distance, center_x, center_y);
            // =========================================================================
        }
    }
}