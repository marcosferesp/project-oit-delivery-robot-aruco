/*
 * ==============================================================================
 * mover.cpp
 * Contains the control architecture to navigate sequences of ArUco markers, 
 * align the physical chassis, and park at designated endpoints
 * Author : Marcos Ferrando España
 * ==============================================================================
 */

#include "mover.hpp"
#include "aruco_follower.hpp"
#include <chrono>

using namespace std::chrono_literals;

// #define DEBUG_SIMPLE_MOVE_PUB
#define DEBUG_COMMENTS 1


#if DEBUG_SIMPLE_MOVE_PUB
/* 
 * Constructor MoverNode
 * Initializes a basic publisher for debugging motor limits
 * Inputs :
 * Output :
 */
MoverNode::MoverNode() : Node("mover_node") {
    // --- Publisher ---
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // --- Timer ---
    timer_ = this->create_wall_timer(500ms, std::bind(&MoverNode::moveCallback, this));

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and sending forward velocity...");
}

/* 
 * Function moveCallback
 * Broadcasts a constant manual velocity for debugging
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


// Parking targets & tolerances
#define X_PARK          960.0
#define Y_PARK          540.0
#define UNCERTAINTY     30.0
#define HYSTERESIS      80.0
#define ANGLE_PARK      0.0
#define ANGLE_UNCE      0.05
#define ANGLE_HYST      0.10

// Proportional gains
#define KP_Y            0.00037
#define KP_X            0.00021
#define KP_ERR_ANGLE    0.127
#define KP_DYN_ANGLE    0.001

// Motor limits
#define SPEED_LINEAR    0.4
#define SPEED_ANGULAR   0.4

// Timeout
#define ARUCO_TIMEOUT   1.0
#define WAIT_TIMEOUT    5.0


/* 
 * Constructor ArucoFollowerNode
 * Initializes network connections and injects the route sequences into memory
 * Inputs : dest_id (int16_t) : The final ArUco ID requested from the console
 * Output :
 */
ArucoFollowerNode::ArucoFollowerNode(int16_t dest_id) : Node("aruco_follower_node"), rs(ROBOT_IDLE) {
    // --- Publisher ---
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);   // Broadcast velocity commands to the hardware motors
    
    // --- Subscriber ---
    aruco_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
        "/aruco_coordinates", 10, std::bind(&ArucoFollowerNode::arucoCallback, this, std::placeholders::_1));   // Unified topic: x, y, z(angle), and w(ID) synchronized
    
    taxi_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/cmd_taxi", 10, std::bind(&ArucoFollowerNode::taxiCallback, this, std::placeholders::_1));         
    
    // Trigger the main motor control loop at 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&ArucoFollowerNode::ctrlLoop, this));

    target_id = -1; 
    target_x = 0.0;
    target_y = 0.0;
    target_angle = 0.0;
    corner_turn = false;

    time = this->now();

    // --- Route Database Init ---
    route.push_back({5, false, 2.0, 15.0,  3, false}); 
    route.push_back({3, false, 2.0, 15.0,  4, false}); 
    route.push_back({4, true,  0.0, 15.0, -1, false});

    // route.push_back({2, false, 2.0, 15.0,  0, false});
    // route.push_back({0, false, 4.0, 30.0,  1, false});
    // route.push_back({1, false, 1.5, 15.0,  6, false});
    // route.push_back({6, false, 4.0, 30.0,  7, false});
    // route.push_back({7, false, 1.5, 15.0,  0, false});

    // Test Static Queue
    // rteQue.push(1);
    // rteQue.push(0);

    bool dest_found = false;
    for( size_t i=0; i<route.size(); i++ ){
        if (route[i].aruco_id == dest_id) {
            // Found the user's request and force it to be the final destination
            route[i].is_destination = true;
            dest_found = true;
        } else {
            // Ensure no other marker in the map accidentally triggers the parking state
            route[i].is_destination = false; 
        }
    }

    // Abort if the user typed an ID that doesn't exist in the map
    if (!dest_found) {
        RCLCPP_ERROR(this->get_logger(), "FATAL: Destination ID %d does not exist in the route database.", dest_id);
        throw std::runtime_error("Invalid destination ID requested.");
    }

#if DEBUG_COMMENTS
    RCLCPP_INFO(this->get_logger(), "==== ROUTE DATABASE ====");
    for (size_t i = 0; i < route.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Index [%zu] -> ID: %2d | Dest: %c | Dist: %.1f | Timeout: %4.1fs | Next ID: %2d",
            i, route[i].aruco_id, route[i].is_destination ? 'Y':'N', route[i].dist_to_next, route[i].search_timeout, route[i].next_id);
    }
#endif // DEBUG_COMMENTS

    RCLCPP_INFO(this->get_logger(), "Motor Ctrl is online and preparing to follow ArUco nearby...");
}

/*
 * Function arucoCallback
 * Processes unified ArUco data (ID + Coordinates) to prevent network desynchronization
 * Inputs : msg (geometry_msgs::msg::Quaternion::SharedPtr) : x, y, z(angle), w(id)
 */
void ArucoFollowerNode::arucoCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    int incoming_id = static_cast<int>(msg->w);
    float incoming_x = msg->x;
    float incoming_y = msg->y;
    float incoming_angle = msg->z;

    bool accept_marker = false;
    int previous_id = target_id;

    bool id_exists_in_db = false;
    bool id_visited = false;
    int expected_next = -1;
    
    // Poll the database to check if the incoming ID exists in our planned route
    for (size_t i = 0; i < route.size(); i++) {
        if (route[i].aruco_id == incoming_id) {
            id_exists_in_db = true;
            id_visited = route[i].visited;
        }
        // Look up the expected next destination based on our current target
        if (route[i].aruco_id == target_id) {
            expected_next = route[i].next_id; 
        }
    }

    // Drop the frame instantly if the camera hallucinates an unregistered ID or sees one it already visited
    if (!id_exists_in_db || id_visited) {
        RCLCPP_INFO(this->get_logger(), "REJECTED: ID %d %s", incoming_id, (!id_exists_in_db)?"INEXISTENT":"VISITED");
        return;
    } 

    if (rs == ROBOT_SEARCH) {
        // Search State: Only accept the planned next destination, OR recover the previous ID if the camera blinked
        if (incoming_id == expected_next || incoming_id == target_id) {
            target_id = incoming_id;
            accept_marker = true;
        }
    } 
    else if (rs == ROBOT_IDLE) {
        // Booting State: Accept any valid ID to begin a run
        target_id = incoming_id;
        accept_marker = true;
    } 
    else {
        // Locked State (Move/Park): Reject everything except our active target
        if (incoming_id == target_id) {
            accept_marker = true;
        }
        // Accept the next destination if it appears before the current one disappears under some conditions
        else if (incoming_id == expected_next) {
            // Do not accept the next marker unless we have physically arrived at the current one
            bool y_close = (std::abs(Y_PARK - target_y) < 80.0);
            
            // The robot has to finish pivoting
            float cra = ANGLE_PARK - target_angle;
            cra = atan2(sin(cra), cos(cra));
            bool has_turned = (std::abs(cra) < 0.05);
            
            // It is safe to switch targets if we are not in a corner or if the corner turn is completely finished
            bool safe_switch = (!corner_turn || has_turned);

            RCLCPP_INFO(this->get_logger(), "Next ID %d spotted! y_close: %c | has_turned: %c | safe_switch: %c", 
                incoming_id, y_close ? 'Y':'N', has_turned ? 'Y':'N', safe_switch ? 'Y':'N');

            if (y_close && safe_switch) {
                target_id = incoming_id;
                accept_marker = true;
            }
        }
    }

    // If the ID is approved, permanently commit the buffered coordinates to memory
    if (accept_marker) {
        // If the robot accepts a new ID it resets the corner latch
        if (previous_id != incoming_id && previous_id != -1) {
            corner_turn = false;
            for (size_t i = 0; i < route.size(); i++) {
                if (route[i].aruco_id == previous_id) {
                    route[i].visited = true;
                    RCLCPP_INFO(this->get_logger(), "ID %d officially marked VISITED.", previous_id);
                    break;
                }
            }
        }
        
        // Because the ID and the coordinates arrived in the exact same packet, cross-contamination is physically impossible.
        target_x = incoming_x;
        target_y = incoming_y;
        target_angle = incoming_angle;
        
        time = this->now(); 
    }
}

/*
 * Function taxiCallback
 * Receives live terminal inputs, validates them with the database and pushes valid IDs into the FIFO queue.
 */
void ArucoFollowerNode::taxiCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    int new_id = msg->data;
    bool is_valid = false;

    // Validate the input with the route database
    for (size_t i = 0; i < route.size(); i++) {
        if (route[i].aruco_id == new_id) {
            is_valid = true;
            break;
        }
    }

    if (is_valid) {
        rteQue.push(new_id);
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[LIVE COMMAND] ID %d added to the FIFO Queue!\033[0m", new_id);
    } else {
        RCLCPP_WARN(this->get_logger(), "\033[1;31m[LIVE COMMAND REJECTED] ID %d does not exist in the map.\033[0m", new_id);
    }
}

/*
 * Function resetRoute
 * Resets the visited boolean for all markers in the database
 */
void ArucoFollowerNode::resetRoute() {
    for (size_t i = 0; i < route.size(); i++) {
        route[i].visited = false;
    }
#if DEBUG_COMMENTS
    RCLCPP_INFO(this->get_logger(), "[ROUTE RESET] All route markers have been reset to unvisited.");
#endif
}

/*
 * Function setDest
 * Sets a new valid destination for the robot
 */
void ArucoFollowerNode::setDest(int16_t dest_id) {
    bool dest_found = false;
    for( size_t i=0; i<route.size(); i++ ){
        if (route[i].aruco_id == dest_id) {
            // Found the user's request and force it to be the final destination
            route[i].is_destination = true;
            dest_found = true;
        } else {
            // Ensure no other marker in the map accidentally triggers the parking state
            route[i].is_destination = false; 
        }
    }

    // Abort if the user typed an ID that doesn't exist in the map
    if (!dest_found) {
        RCLCPP_ERROR(this->get_logger(), "FATAL: Destination ID %d does not exist in the route database.", dest_id);
        throw std::runtime_error("Invalid destination ID requested.");
    }
}

/* 
 * Function ctrlLoop
 * Evaluates the robot's current state and sends motor commands
 * Inputs :
 * Output :
 */
void ArucoFollowerNode::ctrlLoop() {
    auto message = geometry_msgs::msg::Twist();
    auto now = this->now();

    // If 1 second passes with no new data we lost the marker
    bool aruco_visible = ((target_id != -1) && ((now - time).seconds() < ARUCO_TIMEOUT));

    // Database polling
    robot_route_t active_route;
    bool route_found = false;
    
    // Continuously synchronize our local variables with the active target's database entry
    for (size_t i = 0; i < route.size(); i++) {
        if (route[i].aruco_id == target_id) {
            active_route = route[i];
            route_found = true;
            break;
        }
    }
    
    // Prevent memory faults
    if (!route_found) {
        active_route = { (uint8_t)target_id, false, 0.0, 5.0, -1, false };
    }

#if DEBUG_COMMENTS
    RCLCPP_INFO(this->get_logger(), "Active Route -> Dest: %c | Timeout: %.1fs | Next ID: %d", 
                active_route.is_destination ? 'Y':'N', active_route.search_timeout, active_route.next_id);

    RCLCPP_INFO(this->get_logger(), "==== MOVER INPUTS ====");
    RCLCPP_INFO(this->get_logger(), "Target -> ID: %d | is_Dest: %c | Vis: %c", target_id, active_route.is_destination ? 'Y':'N', aruco_visible ? 'Y':'N');
    RCLCPP_INFO(this->get_logger(), "Coords -> X: %.1f | Y: %.1f | Ang: %.4f", target_x, target_y, target_angle);
#endif // DEBUG_COMMENTS

    // Calculate physical pixel distance between the robot and the target coordinate
    float error_y = Y_PARK - target_y;
    float error_x = X_PARK - target_x;

    // Calculate pure physical alignment to the ArUco marker
    float raw_angle = ANGLE_PARK - target_angle;
    raw_angle = atan2(sin(raw_angle), cos(raw_angle));

    // Isolate X-correction math purely to destination parking maneuvers
    float dyn_x = (!active_route.is_destination) ? 0.0 : error_x;

    // Generate an optimal curve angle to merge back to the center line
    float dyn_angle = ANGLE_PARK - atan(KP_DYN_ANGLE*dyn_x);

    // Calculate the difference between where the robot is pointing and where the dynamic wants it to point
    float error_angle = dyn_angle - target_angle;
    error_angle = atan2(sin(error_angle), cos(error_angle));

#if DEBUG_COMMENTS
    RCLCPP_INFO(this->get_logger(), "==== MOVER MATH ====");
    RCLCPP_INFO(this->get_logger(), "Errors -> ErrX: %.1f | ErrY: %.1f", error_x, error_y);
    RCLCPP_INFO(this->get_logger(), "Angles -> raw_ang: %.4f | dyn_x: %.1f | dyn_ang: %.4f | err_ang: %.4f", raw_angle, dyn_x, dyn_angle, error_angle);
#endif

    // State condition flags
    bool ready_to_move, ready_to_park, robot_drifted;

    if (active_route.is_destination) {
        // Destinations only evaluate physical straightness (raw_angle) to park
        ready_to_move = ((std::abs(error_y) > UNCERTAINTY) || (std::abs(raw_angle) > ANGLE_UNCE));
        ready_to_park = ((std::abs(error_y) < UNCERTAINTY) && (std::abs(raw_angle) < ANGLE_UNCE));
        robot_drifted = ((std::abs(error_y) > HYSTERESIS) || (std::abs(raw_angle) > ANGLE_HYST));
    } else {
        // Pass-throughs continue to use error_angle and error_x
        ready_to_move = ((std::abs(error_y) > UNCERTAINTY) || (std::abs(error_x) > UNCERTAINTY) || (std::abs(error_angle) > ANGLE_UNCE));
        ready_to_park = false; 
        robot_drifted = ((std::abs(error_y) > HYSTERESIS) || (std::abs(error_x) > HYSTERESIS) || (std::abs(error_angle) > ANGLE_HYST));
    }

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
            } else {
                rs = ROBOT_WAIT;
                wait_time = now;
            }
            break;

        case ROBOT_SEARCH:
            if (aruco_visible) {
                rs = ROBOT_MOVE;
                RCLCPP_INFO(this->get_logger(), "Target ID %d acquired from search. Moving.", target_id);
            } else if ((now - search_start_time).seconds() > active_route.search_timeout) {
                rs = ROBOT_IDLE;
                RCLCPP_INFO(this->get_logger(), "Search timeout! Target lost. Idling.");
            }
            break;

        case ROBOT_WAIT:
        {
            int dest_id = 5;

            if (robot_drifted) {
                // Realignment if bumped from its parking position
                rs = ROBOT_MOVE;
                break;

            } else if ((now - wait_time).seconds() > WAIT_TIMEOUT) {
                // Timer expired : get back to default position
                setDest(dest_id);
                resetRoute();
                rs = ROBOT_MOVE; 
                break;
            }

            if (!rteQue.empty()) {
                dest_id = rteQue.front();
                rteQue.pop();
                RCLCPP_INFO(this->get_logger(), "\033[1;36mFIFO Queue element found : ID %d\033[0m", dest_id);

                // Print the remaining queue (temporary clone of the queue)
                std::queue<int> temp_q = rteQue;
                std::string temp_qstr = "[ ";
                while (!temp_q.empty()) {
                    temp_qstr += std::to_string(temp_q.front()) + " ";
                    temp_q.pop();
                }
                temp_qstr += "]";
                
                RCLCPP_INFO(this->get_logger(), "\033[1;36mRemaining in Queue: %s\033[0m", temp_qstr.c_str());

                RCLCPP_INFO(this->get_logger(), "==== ROUTE DATABASE ====");
                for (size_t i = 0; i < route.size(); i++) {
                    RCLCPP_INFO(this->get_logger(), "Index [%zu] -> ID: %2d | Dest: %c | Dist: %.1f | Timeout: %4.1fs | Next ID: %2d",
                        i, route[i].aruco_id, route[i].is_destination ? 'Y':'N', route[i].dist_to_next, route[i].search_timeout, route[i].next_id);
                }

                // Set the new destination ID
                setDest(dest_id);
                
                // Wipe the visited locations booleans
                resetRoute();

                rs = ROBOT_MOVE; 
                break;
            }
            break;
        }

        default:
            break;
    }

    // Default motor state is locked to 0
    message.linear.x = 0.0;
    message.angular.z = 0.0;

    float angle_brake = 0.0, x_brake = 0.0;

    switch (rs) {
        case ROBOT_MOVE:
        {
            if (active_route.is_destination) {  // Destination
                // Drive forward proportionally to the Y-error
                message.linear.x = KP_Y * error_y;

                if( std::abs(error_y) <= UNCERTAINTY ){
                    // If extremely close to parking spot, force wheels straight
                    message.angular.z = -(KP_ERR_ANGLE * raw_angle);
                }else{
                    // If driving toward spot, steer using the dynamic curve
                    message.angular.z = -(KP_ERR_ANGLE * error_angle);
                }
            } else {    // Pass-through
                if (std::abs(error_y) < 80) {
                    corner_turn = true;
                }
                if (!corner_turn) {
                    // We advance to correct Y coordinate as much possible before aligning
                    message.angular.z = 0.0;
                    if (error_y > 0.0) {
                        message.linear.x = SPEED_LINEAR;   // Marker is ahead
                    } else {
                        message.linear.x = -SPEED_LINEAR;  // Marker is behind
                    }
                } else {
                    // Uses raw_angle explicitly because we only care about being parallel to the hallway
                    message.angular.z = -(KP_ERR_ANGLE * raw_angle);

                    // Instantly kill forward movement if angular threshold is too high
                    if (std::abs(raw_angle) > 0.05) {
                        message.linear.x = 0.0;
                        RCLCPP_INFO(this->get_logger(), "Brake -> abs(raw) = %.4f > 0.05", std::abs(raw_angle));
                    } else {
                        // Uses the raw angle so it only drives fast when pointing perfectly straight
                        angle_brake = std::pow(std::cos(raw_angle), 5.0);
                        // Slows down the forward speed by up to 70% if the robot is far off-center
                        x_brake = std::max(0.3f, 1.0f - (std::abs(error_x) / 500.0f));
                        message.linear.x = SPEED_LINEAR * std::max(0.0f, angle_brake) * x_brake;
                    }
                }
            }
            
            // Exceeding physical robot capabilities
            if (message.linear.x > SPEED_LINEAR) message.linear.x = SPEED_LINEAR;
            if (message.linear.x < -SPEED_LINEAR) message.linear.x = -SPEED_LINEAR;
            if (message.angular.z > SPEED_ANGULAR) message.angular.z = SPEED_ANGULAR;
            if (message.angular.z < -SPEED_ANGULAR) message.angular.z = -SPEED_ANGULAR;

            break;
        }

        case ROBOT_SEARCH:
        {
            message.linear.x = SPEED_LINEAR;
            message.angular.z = 0.0;
            break;
        }

        default:
            break;
            
    }

// #if DEBUG_COMMENTS
    const char* state_str[] = {"IDLE", "MOVE", "PARK", "SEARCH", "WAIT"};
    RCLCPP_INFO(this->get_logger(), 
        "STATE: %-6s | ID: %2d (Dest:%c) | Vis: %c | ErrX: %5.0f | ErrY: %5.0f | Ang: %5.2f | Lin: %4.2f | AngZ: %5.2f |", 
        state_str[rs], target_id, active_route.is_destination ? 'T' : 'F', aruco_visible ? 'Y' : 'N', error_x, error_y, error_angle, message.linear.x, message.angular.z);
// #endif // DEBUG_COMMENTS
    
    cmd_pub_->publish(message);
}

/* 
 * Main
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 

    // Extract non-ROS arguments from the console command
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

#if DEBUG_SIMPLE_MOVE_PUB

    rclcpp::spin(std::make_shared<MoverNode>());

#else

    // Abort if user forgot ID argument
    if (args.size() < 2) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Launch aborted: No destination ID provided.");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: ros2 run aruco_vision mover_node <destination_id>");
        rclcpp::shutdown();
        return 1;
    }

    // Default destination if the user types nothing
    int16_t dest_id = -1;
    
    try {
        dest_id = static_cast<int16_t>(std::stoi(args[1]));
    } catch (const std::invalid_argument& e) {
        // Abort if the user typed letters instead of a number
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Launch aborted: Destination ID must be a valid number.");
        rclcpp::shutdown();
        return 1;
    }

    try {
        // Attempt to build and spin the node with the validated ID
        rclcpp::spin(std::make_shared<ArucoFollowerNode>(dest_id));
    } catch (const std::runtime_error& e) {
        // If the ID is a valid number but doesn't exist in our map, safely abort
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Aborting launch: %s", e.what());
    }
    
#endif // DEBUG_SIMPLE_MOVE_PUB

    rclcpp::shutdown();

    return 0;
}