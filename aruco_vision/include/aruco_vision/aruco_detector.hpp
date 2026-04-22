#ifndef ARUCO_VISION__ARUCO_DETECTOR_HPP_
#define ARUCO_VISION__ARUCO_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode();

private:
    // Callbacks
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Network subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // Variables to store hardware data
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool got_camera_info_;
};

#endif  // ARUCO_VISION__ARUCO_DETECTOR_HPP_