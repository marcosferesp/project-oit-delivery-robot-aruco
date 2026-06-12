/*
 * ==============================================================================
 * aruco_detector.cpp
 * Contains the vision processing logic, translating camera images to find ArUco 
 * markers and calculating their physical orientation and coordinates.
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "aruco_detector.hpp"

#define DEBUG_COMMENTS 1

/* 
 * Constructor ArucoDetectorNode
 * Initializes ROS 2 network connections for video processing
 * Inputs :
 * Output :
 */
ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector_node"), got_camera_info_(false) {
    // --- Subscriptions ---
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10, std::bind(&ArucoDetectorNode::infoCallback, this, std::placeholders::_1));  // Listen to the camera's physical lens calibration data
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));   // Listen to the raw video stream broadcasted by the camera

    // --- Publishers ---
    // coord_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/aruco_coordinates", 10);   // Broadcast the 2D pixel coordinates and calculated rotation angle
    // id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/aruco_id", 10);                    // Broadcast the ID number of the recognized ArUco marker
    coord_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/aruco_coordinates", 10);

    RCLCPP_INFO(this->get_logger(), "ArUco Brain is online and waiting for video...");
}

/* 
 * Function infoCallback
 * Saves physical camera calibration data to internal OpenCV matrices
 * Inputs : msg (sensor_msgs::msg::CameraInfo::SharedPtr) : the .yaml calibration data
 * Output :
 */
void ArucoDetectorNode::infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Ignore incoming messages if the calibration is already locked in memory
    if (got_camera_info_) return; 

    // Reject empty or invalid calibration matrices to prevent crashes
    if (msg->k[0] == 0.0) {
        RCLCPP_WARN(this->get_logger(), "Warning: Camera sent empty calibration data. Waiting...");
        return;
    }

    // Convert the 1D ROS 2 focal array into a 3x3 OpenCV matrix
    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
    
    // Extract lens distortion coefficients if the camera provides them
    if (msg->d.size() > 0) {
        dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, (void*)msg->d.data()).clone();
    } else {
        // Fallback: Fill the distortion matrix with zeros to satisfy OpenCV math requirements
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }
    
    // Lock the calibration state to allow video frames to begin processing
    got_camera_info_ = true;
    RCLCPP_INFO(this->get_logger(), "Calibration loaded successfully.");
}

/* 
 * Function imageCallback
 * Scans video frames to detect ArUco markers and calculates 2D vectors and distances
 * Inputs : msg (sensor_msgs::msg::Image::SharedPtr) : the raw video frame from the camera
 * Output :
 */
void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Block image processing until camera physics are loaded
    if (!got_camera_info_) return;

    cv_bridge::CvImagePtr cv_ptr;   
    try {
        // Translate the ROS 2 network frame into a BGR 8-bit OpenCV image matrix
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV translation failed!");
        return;
    }

    // Load the official dictionary for 4x4 ArUco markers up to 50 IDs
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // Scan the image matrix to find squares and decode their internal grid IDs
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds);

    float center_x = 0.0, center_y = 0.0, dx = 0.0, dy = 0.0, marker_angle = 0.0;
    float c0x = 0.0, c1x = 0.0, c2x = 0.0, c3x = 0.0;
    float c0y = 0.0, c1y = 0.0, c2y = 0.0, c3y = 0.0;
    float top_mid_x = 0.0, top_mid_y = 0.0, bot_mid_x = 0.0, bot_mid_y = 0.0;

    // geometry_msgs::msg::Point coord_msg;
    // std_msgs::msg::Int32 id_msg;

    geometry_msgs::msg::Quaternion coord_msg;

    // Only process physics if at least one valid marker was found
    if (markerIds.size() > 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        
        // Estimate the 3D position of the marker assuming a 10cm physical size
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.10, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // Process data for every marker detected in the current frame
        for (size_t i = 0; i < markerIds.size(); i++) {

            // Extract X and Y pixel coordinates for all 4 corners of the marker
            c0x = markerCorners[i][0].x, c0y = markerCorners[i][0].y;
            c1x = markerCorners[i][1].x, c1y = markerCorners[i][1].y;
            c2x = markerCorners[i][2].x, c2y = markerCorners[i][2].y;
            c3x = markerCorners[i][3].x, c3y = markerCorners[i][3].y;

            // Average the 4 corners to find the exact center pixel of the marker
            center_x = (c0x + c1x + c2x + c3x) / 4.0;
            center_y = (c0y + c1y + c2y + c3y) / 4.0;
            coord_msg.x = center_x;
            coord_msg.y = center_y;

            // Find the midpoints of the top and bottom edges to ignore perspective stretching
            top_mid_x = (c0x + c1x) / 2.0;
            top_mid_y = (c0y + c1y) / 2.0;
            bot_mid_x = (c2x + c3x) / 2.0;
            bot_mid_y = (c2y + c3y) / 2.0;
            // Calculate the 2D directional vector pointing from the bottom to the top
            dx = top_mid_x - bot_mid_x;
            dy = top_mid_y - bot_mid_y;

            // Use atan2 to determine the exact absolute 2D rotational angle in radians
            marker_angle = atan2(dx, -dy);
            coord_msg.z = marker_angle;

            // Send the ID of this specific marker to the mover node
            coord_msg.w = markerIds[i];

            // Send coordinates to the mover node
            coord_pub_->publish(coord_msg);
            
            // // Send the ID of this specific marker to the mover node
            // id_msg.data = markerIds[i];
            // id_pub_->publish(id_msg);

#if DEBUG_COMMENTS
            // Extract the physical depth distance (Z-axis translation vector)
            double distance = tvecs[i][2];
            RCLCPP_INFO(this->get_logger(), "==== DETECTOR CALCS [ID: %d] ====", markerIds[i]);
            RCLCPP_INFO(this->get_logger(), "Corners -> TL: %.1f,%.1f | TR: %.1f,%.1f | BR: %.1f,%.1f | BL: %.1f,%.1f", c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y);
            RCLCPP_INFO(this->get_logger(), "Midpts  -> TOP: %.1f,%.1f | BOT: %.1f,%.1f", top_mid_x, top_mid_y, bot_mid_x, bot_mid_y);
            RCLCPP_INFO(this->get_logger(), "Vector  -> dx: %.1f | dy: %.1f", dx, dy);
            RCLCPP_INFO(this->get_logger(), "Output  -> Dist: %.2fm | Cntr: %.1f,%.1f | Angle: %.4f rad", distance, center_x, center_y, marker_angle);
#endif // DEBUG_COMMENTS
        }
    }
}