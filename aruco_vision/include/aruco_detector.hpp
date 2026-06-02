/*
 * ==============================================================================
 * aruco_detector.hpp
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#ifndef ARUCO_VISION__ARUCO_DETECTOR_HPP_
#define ARUCO_VISION__ARUCO_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

/*
 * Class ArucoDetectorNode
 * Inherits from rclcpp::Node to process camera streams and detect ArUco markers
 */
class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode();

private:
    // --- Callbacks ---
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // --- Variables ---
    cv::Mat camera_matrix_; 
    cv::Mat dist_coeffs_;   
    bool got_camera_info_;  
};

#endif  // ARUCO_VISION__ARUCO_DETECTOR_HPP_