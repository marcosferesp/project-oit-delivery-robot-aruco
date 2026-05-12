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
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "std_msgs/msg/float32.hpp"


/* 
 * Class ArucoDetectorNode
 * It inherits from rclcpp::Node to get network communication abilities
 */
class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode();    // Setup function that will run automatically when the node boots

private:
    // Callbacks
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);       // Triggered whenever the camera broadcasts its .yaml calibration data
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);           // Triggered 30 times a second (30 Hz) whenever a new video frame arrives

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;    // Subscribes to the camera's lens calibration data
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;        // Subscribes to the raw video stream
    
    // Variables
    cv::Mat camera_matrix_; // OpenCV focal length and optical center matrix
    cv::Mat dist_coeffs_;   // OpenCV lens distortion curve values matrix
    bool got_camera_info_;  // Prevents image math from running before calibration arrives
};

#endif  // ARUCO_VISION__ARUCO_DETECTOR_HPP_