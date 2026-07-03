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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_PARK,
    ROBOT_SEARCH,
    ROBOT_WAIT
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
    void resetRoute();
    void setDest(int16_t dest_id);
    void declareParameters();

    // --- Callbacks ---
    void arucoCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
    void taxiCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void pkgCallback(const std_msgs::msg::Bool::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &parameters);

    // --- Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr db_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telem_pub_;

    // --- Subscribers ---
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr aruco_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr taxi_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pkg_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr db_list_sub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Dynamic Parameters ---
    double x_park, y_park, uncertainty, hysteresis;
    double angle_park, angle_unce, angle_hyst;
    double kp_y, kp_x, kp_err_angle, kp_dyn_angle;
    double speed_linear, speed_angular;
    double aruco_timeout, wait_timeout, package_timeout, departure_delay;
    
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // --- Variables ---
    robot_state_t rs;
    float target_x;
    float target_y;
    float target_angle;
    int target_id;
    bool corner_turn;
    bool pkg_taken;
    bool depart_to;

    rclcpp::Time time;
    rclcpp::Time search_start_time;
    rclcpp::Time wait_time;
    rclcpp::Time depart_time;

    // --- Database ---
    std::vector<robot_route_t> route;

    // --- FIFO ---
    std::queue<int> rteQue;
};

#endif //ARUCO_VISION__ARUCO_FOLLOWER_HPP_