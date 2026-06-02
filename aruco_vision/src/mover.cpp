/*
 * ==============================================================================
 * mover.cpp
 * Contains the vision processing logic, translating camera images to find ArUco markers and calculating their physical distance
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "mover.hpp"
#include "aruco_follower.hpp"
#include <chrono>

using namespace std::chrono_literals;

// #define DEBUG_SIMPLE_MOVE_PUB


#if DEBUG_SIMPLE_MOVE_PUB
/* 
 * Constructor MoverNode
 * Prepares the node to run by setting up the publisher and starting the timer
 * Inputs :
 * Output :
 */
MoverNode::MoverNode() : Node("mover_node") {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MoverNode::moveCallback, this));

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and sending forward velocity...");
}

/* 
 * Function moveCallback
 * Creates a movement message and broadcasts it to the motors
 * Inputs :
 * Output :
 */
void MoverNode::moveCallback() {
    auto message = geometry_msgs::msg::Twist();

    message.linear.x = 0.1;
    message.angular.z = 0.0;

    cmd_pub_->publish(message);
}
#endif // DEBUG_SIMPLE_MOVE_PUB


/* 
 * Constructor ArucoFollowerNode
 * Init the node and sets up the publisher, subscriber and timer
 * Inputs :
 * Output :
 */
ArucoFollowerNode::ArucoFollowerNode() : Node("aruco_follower_node"), rs(ROBOT_IDLE) {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    coord_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/aruco_coordinates", 10, std::bind(&ArucoFollowerNode::coordCallback, this, std::placeholders::_1));
    id_sub_ = this->create_subscription<std_msgs::msg::Int32>("/aruco_id", 10, std::bind(&ArucoFollowerNode::idCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&ArucoFollowerNode::ctrlLoop, this));

    time = this->now();

    // PHASE 1: DATABASE
    // We use vector push_back so the array index does NOT have to match the ArUco ID.
    // Format: {aruco_id, is_destination, dist_to_next, search_timeout, next_id}
    route.push_back({5, false, 2.0, 15.0,  3}); 
    route.push_back({3, false, 2.0, 15.0,  4}); 
    route.push_back({4, true,  0.0, 15.0, -1});

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and preparing to follow ArUco nearby...");
}

// /* 
//  * Function distanceCallback
//  * Triggers whenever a new distance measurement is published by the camera
//  * Inputs : msg (std_msgs::msg::Float32::SharedPtr) : the distance data
//  * Output :
//  */
// void ArucoFollowerNode::distanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
//     dist = msg->data;
//     time = this->now();
// }

/* 
 * Function coordCallback
 * Triggers whenever new x and y coordinates are published by the camera
 * Inputs : msg (geometry_msgs::msg::Point::SharedPtr) : the coordinates data
 * Output :
 */
void ArucoFollowerNode::coordCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // PHASE 2: THE MEMORY LATCH (Part 1)
    // We do NOT update the robot's real coordinates or the timeout clock yet.
    // We hold them in temporary buffers until the Bouncer approves the ID.
    temp_x = msg->x;
    temp_y = msg->y;
    temp_angle = msg->z;
}

/* 
 * Function coordCallback
 * Triggers whenever new x and y coordinates are published by the camera
 * Inputs : msg (std_msgs::msg::Int32::SharedPtr) : the ArUco ID data
 * Output :
 */
void ArucoFollowerNode::idCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    int incoming_id = msg->data;
    bool accept_marker = false;

    // PHASE 3: SECURITY FILTER (The Bouncer)
    // TIER 1: Does this ID even exist in our universe?
    bool id_exists_in_db = false;
    int expected_next = -1;
    
    for (size_t i = 0; i < route.size(); i++) {
        if (route[i].aruco_id == incoming_id) {
            id_exists_in_db = true; // It is a real database entry
        }
        if (route[i].aruco_id == target_id) {
            expected_next = route[i].next_id; // Look up what the current target expects next
        }
    }

    if (!id_exists_in_db) {
        return; // Firewall catches a hallucination. Drop the frame immediately.
    }

    // TIER 2 & 3: State-based ID filtering
    if (rs == ROBOT_SEARCH) {
        // We are searching. We ONLY accept the next ID we expect (e.g., ID 4), 
        // OR we re-accept the current ID (e.g., ID 3) if the camera merely blinked.
        if (incoming_id == expected_next || incoming_id == target_id) {
            target_id = incoming_id;
            accept_marker = true;
        }
    } 
    else if (rs == ROBOT_IDLE) {
        // Booting up. Accept any valid ID to start the run.
        target_id = incoming_id;
        accept_marker = true;
    } 
    else {
        // We are in MOVE/PARK mode. We MUST ignore everything except our current target ID.
        if (incoming_id == target_id) {
            accept_marker = true;
        } else if (incoming_id == expected_next) {
            // The Smooth Handoff: We see the next destination before fully losing the current one.
            target_id = incoming_id;
            accept_marker = true;
        }
    }

    // PHASE 2: THE MEMORY LATCH (Part 2)
    // The Bouncer approved the ID. Now we lock the coordinates into the robot's memory 
    // and reset the 1-second visibility timeout clock.
    if (accept_marker) {
        target_x = temp_x;
        target_y = temp_y;
        target_angle = temp_angle;
        time = this->now(); 
    }
}

// Target values to park and uncertainties
#define X_PARK          960.0
#define Y_PARK          540.0
#define UNCERTAINTY     30.0
#define HYSTERESIS      80.0
#define ANGLE_PARK      0.0
#define ANGLE_UNCE      0.2
#define ANGLE_HYST      0.35

// Proportional gains
#define KP_Y            0.00037
#define KP_X            0.00021
#define KP_ERR_ANGLE    0.127
#define KP_DYN_ANGLE    0.001

// Motor limits
#define SPEED_LINEAR    0.2
#define SPEED_ANGULAR   0.4
#define DEADBAND_LINEAR 0.01
#define DEADBAND_ANGLE  0.01

// Sensor time out
#define ARUCO_TIMEOUT   1.0

/* 
 * Function ctrlLoop
 * Evaluates the robot's current state and sends motor commands
 * Inputs :
 * Output :
 */
void ArucoFollowerNode::ctrlLoop() {
    auto message = geometry_msgs::msg::Twist();
    auto now = this->now();

    // If 1 second passes with no new data, we lost the marker
    bool aruco_visible = (now - time).seconds() < ARUCO_TIMEOUT;

    // PHASE 1 & 2: THE POLLING SYSTEM & MEMORY
    // We poll the database every loop to find the data for our latched target_id.
    // If we go blind, target_id stops changing, meaning active_route PERMANENTLY holds 
    // the search_timeout and next_id of the marker we just left!
    robot_route_t active_route;
    bool route_found = false;
    
    for (size_t i = 0; i < route.size(); i++) {
        if (route[i].aruco_id == target_id) {
            active_route = route[i];
            route_found = true;
            break;
        }
    }
    
    // Safety fallback just in case
    if (!route_found) {
        active_route = { (uint8_t)target_id, false, 0.0, 5.0, -1 };
    }

    RCLCPP_INFO(this->get_logger(), "==== MOVER INPUTS ====");
    RCLCPP_INFO(this->get_logger(), "Target -> ID: %d | is_Dest: %c | Vis: %c", target_id, active_route.is_destination ? 'Y':'N', aruco_visible ? 'Y':'N');
    RCLCPP_INFO(this->get_logger(), "Coords -> X: %.1f | Y: %.1f | Ang: %.4f", target_x, target_y, target_angle);

    float error_y = Y_PARK - target_y;
    float error_x = X_PARK - target_x;

    float raw_angle = ANGLE_PARK - target_angle;
    raw_angle = atan2(sin(raw_angle), cos(raw_angle));

    float dyn_x = (!active_route.is_destination) ? 0.0 : error_x;
    float dyn_angle = ANGLE_PARK - atan(KP_DYN_ANGLE*dyn_x);

    float error_angle = dyn_angle - target_angle;
    error_angle = atan2(sin(error_angle), cos(error_angle));

    RCLCPP_INFO(this->get_logger(), "==== MOVER MATH ====");
    RCLCPP_INFO(this->get_logger(), "Errors -> ErrX: %.1f | ErrY: %.1f", error_x, error_y);
    RCLCPP_INFO(this->get_logger(), "Angles -> raw_ang: %.4f | dyn_x: %.1f | dyn_ang: %.4f | err_ang: %.4f", raw_angle, dyn_x, dyn_angle, error_angle);

    bool ready_to_move = ((std::abs(error_y) > UNCERTAINTY) || (std::abs(error_x) > UNCERTAINTY) || (std::abs(error_angle) > ANGLE_UNCE));
    bool ready_to_park = (active_route.is_destination && ((std::abs(error_y) < UNCERTAINTY) && (std::abs(error_x) < UNCERTAINTY) && (std::abs(error_angle) < ANGLE_UNCE)));
    bool robot_drifted = ((std::abs(error_y) > HYSTERESIS) || (std::abs(error_x) > HYSTERESIS) || (std::abs(error_angle) > ANGLE_HYST));

    // PHASE 4: STATE MACHINE FLOW
    switch (rs) {
        case ROBOT_IDLE:
            if( aruco_visible ){
                if( ready_to_move || robot_drifted ){
                    rs = ROBOT_MOVE;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Moving to align.");
                }else if( ready_to_park ){
                    rs = ROBOT_PARK;
                    RCLCPP_INFO(this->get_logger(), "Target acquired. Parking.");
                }
            }
            break;

        case ROBOT_MOVE:
            if (!aruco_visible) {
                // (YOUR POINT 1): If we lost sight and it WAS the destination, we stop.
                // If it was a pass-through marker, we execute the blind sprint search.
                if (!active_route.is_destination) {
                    rs = ROBOT_SEARCH;
                    search_start_time = now;
                    RCLCPP_INFO(this->get_logger(), "Marker passed. Entering SEARCH for ID %d. Timeout: %.1fs", active_route.next_id, active_route.search_timeout);
                } else {
                    rs = ROBOT_IDLE;
                    RCLCPP_INFO(this->get_logger(), "Arrived at destination but lost sight. Stopping.");
                }
            } else if( ready_to_park ){
                rs = ROBOT_PARK;
                RCLCPP_INFO(this->get_logger(), "Aligning complete. Parking.");
            }
            break;

        case ROBOT_PARK:
            if (!aruco_visible) {
                rs = ROBOT_IDLE;
            } else if( robot_drifted ){
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Robot has drifted. Moving to align.");
            }
            break;

        case ROBOT_SEARCH:
            if (aruco_visible) {
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Target ID %d acquired from search. Moving.", target_id);
            } else if ((now - search_start_time).seconds() > active_route.search_timeout) {
                // (YOUR POINT 2): We use the timeout of the active_route (the marker we just left)
                rs = ROBOT_IDLE;
                RCLCPP_INFO(this->get_logger(), "Search timeout! Target lost. Idling.");
            }
            break;

        default:
            break;
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;

    float angle_brake = 0.0, x_brake = 0.0;

    switch (rs) {
        case ROBOT_MOVE:
        {
            if (active_route.is_destination) {
                message.linear.x = KP_Y * error_y;

                if( std::abs(error_y) <= UNCERTAINTY ){
                    message.angular.z = -(KP_ERR_ANGLE * raw_angle);
                }else{
                    message.angular.z = -(KP_ERR_ANGLE * error_angle);
                }
            } else {
                // PASS-THROUGH ALIGNMENT MATH
                // Uses raw_angle explicitly because we only care about being parallel to the hall.
                message.angular.z = -(KP_ERR_ANGLE * raw_angle);

                if (std::abs(raw_angle) > 0.05) {
                    message.linear.x = 0.0;
                    RCLCPP_INFO(this->get_logger(), "ACTION -> Brake TRIGGERED! abs(raw) = %.4f > 0.05", std::abs(raw_angle));
                } else {
                    // Uses the raw angle so it only drives fast when pointing perfectly straight
                    angle_brake = std::pow(std::cos(raw_angle), 5.0);
                    // Slows down the forward speed by up to 70% if the robot is far off-center
                    x_brake = std::max(0.3f, 1.0f - (std::abs(error_x) / 500.0f));
                    message.linear.x = SPEED_LINEAR * std::max(0.0f, angle_brake) * x_brake;
                    RCLCPP_INFO(this->get_logger(), "ACTION -> Quasi-aligned! Cosine brake active: %.4f", angle_brake);
                }
            }
            
            if (message.linear.x > SPEED_LINEAR) message.linear.x = SPEED_LINEAR;
            if (message.linear.x < -SPEED_LINEAR) message.linear.x = -SPEED_LINEAR;
            if (message.angular.z > SPEED_ANGULAR) message.angular.z = SPEED_ANGULAR;
            if (message.angular.z < -SPEED_ANGULAR) message.angular.z = -SPEED_ANGULAR;

            const char* state_str[] = {"IDLE", "MOVE", "PARK", "SEARCH"};
            RCLCPP_INFO(this->get_logger(), 
                "STATE: %-6s | ID: %2d (Dest:%c) | Vis: %c | ErrX: %5.0f | ErrY: %5.0f | Ang: %5.2f | Lin: %4.2f | AngZ: %5.2f |", 
                state_str[rs], target_id, active_route.is_destination ? 'T' : 'F', aruco_visible ? 'Y' : 'N', error_x, error_y, error_angle, message.linear.x, message.angular.z);

            break;
        }

        case ROBOT_SEARCH:
        {
            // (YOUR POINT 4): The Blind Sprint. Guaranteed parallel from the previous alignment phase.
            message.linear.x = SPEED_LINEAR;
            message.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "FINAL STATE: SEARCH | Sprinting Blindly at Lin: %4.4f", message.linear.x);
            break;
        }

        default:
            break;
            
    }
    
    cmd_pub_->publish(message);
}

/* 
 * Main
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

#if DEBUG_SIMPLE_MOVE_PUB
    rclcpp::spin(std::make_shared<MoverNode>());
#else
    rclcpp::spin(std::make_shared<ArucoFollowerNode>());
#endif // DEBUG_SIMPLE_MOVE_PUB

    rclcpp::shutdown();

    return 0;
}