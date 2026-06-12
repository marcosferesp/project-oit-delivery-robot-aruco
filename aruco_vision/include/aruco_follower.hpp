/*
 * ==============================================================================
 * aruco_follower.hpp
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#ifndef ARUCO_VISION__ARUCO_FOLLOWER_HPP_
#define ARUCO_VISION__ARUCO_FOLLOWER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_PARK,
    ROBOT_SEARCH
} robot_state_t;

typedef struct {
    int16_t aruco_id;
    bool    is_destination;
    float   dist_to_next;
    float   search_timeout;
    int16_t next_id;
    bool    visited;
} robot_route_t;

/*
 * Class ArucoFollowerNode
 * Inherits from rclcpp::Node to command chassis motors based on ArUco vision logic
 */
class ArucoFollowerNode : public rclcpp::Node {
public:
    ArucoFollowerNode(int16_t dest_id);

private:
    // --- Core Logic ---
    void ctrlLoop();

    // --- Callbacks ---
    // void coordCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    // void idCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void arucoCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg);

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // --- Subscribers ---
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coord_sub_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr id_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr aruco_sub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Variables ---
    robot_state_t rs;
    float target_x;
    float target_y;
    float target_angle;
    int target_id;
    bool corner_turn;

    rclcpp::Time time;
    rclcpp::Time search_start_time;

    // --- Buffer Variables --- : Temporary hold for camera data until ID validation is complete
    // float temp_x;
    // float temp_y;
    // float temp_angle;

    // --- Database ---
    std::vector<robot_route_t> route;
};

#endif //ARUCO_VISION__ARUCO_FOLLOWER_HPP_