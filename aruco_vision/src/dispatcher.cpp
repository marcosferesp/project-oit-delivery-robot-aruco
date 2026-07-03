#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <csignal>
#include <readline/readline.h>
#include <readline/history.h>

// --- GLOBAL DISPATCHER STATE ---
bool current_pkg_status = false;
std::string current_robot_pos = "UNKNOWN (Requires Telemetry Node)";
std::vector<int> valid_ids;

// --- POSIX SIGNAL HANDLER ---
void sigintHandler(int signum) {
    std::cout << "\n\033[1;33m[SYSTEM] Shutting down Dispatcher...\033[0m\n";
    rclcpp::shutdown();
    exit(signum); // Brutally forces the OS to reclaim the frozen readline thread
}

/*
 * Function cmdHelp
 * Prints the formatted UI menus and parameter dictionaries
 */
void cmdHelp(const std::string& arg) {
    if (arg.empty()) {
        std::cout << "\n=== AVAILABLE COMMANDS ===\n";
        std::cout << "  taxi  : Manage destinations\n";
        std::cout << "  pkg   : Manage package status\n";
        std::cout << "  param : Live-tune physics & limits\n";
        std::cout << "\n==========================\n";
        std::cout << " Type 'help' <cmd> for more details on a command.\n";

    } else if (arg == "taxi") {
        std::cout << "Objective : Dispatches the robot to a known destinations or lists available targets.\n";
        std::cout << "  taxi list     : Show available destinations\n";
        std::cout << "  taxi <id>     : Dispatch robot to a specific destination\n";

    } else if (arg == "pkg") {
        std::cout << "Objective : Simulates the web-server package retrieval signal.\n";
        std::cout << "  pkg true      : Set package as taken\n";
        std::cout << "  pkg false     : Set package as not taken\n";
        std::cout << "  pkg status    : Show package status and robot position\n";

    } else if (arg == "param") {
        std::cout << "Objective : Edit the robot's physical variables live without recompiling.\n";
        std::cout << "Usage     : param <name> <value>\n";
        std::cout << "          : param list (Shows current values)\n\n";
        std::cout << "--- POSITION & TOLERANCE ---\n";
        std::cout << "  xpark    : X center line (px)\n";
        std::cout << "  ypark    : Y parking depth (px)\n";
        std::cout << "  unce     : Distance uncertainty threshold (px)\n";
        std::cout << "  hyst     : Hysteresis drift limit (px)\n";
        std::cout << "  angpark  : Perfect parking angle (rad)\n";
        std::cout << "  angunce  : Straightness uncertainty threshold (rad)\n";
        std::cout << "  anghyst  : Rotation drift limit (rad)\n\n";
        std::cout << "--- PID GAINS ---\n";
        std::cout << "  kpy      : Proportional speed on Y axis\n";
        std::cout << "  kpx      : Proportional steer on X axis\n";
        std::cout << "  kperrang : Steering aggression toward target angle\n";
        std::cout << "  kpdynang : Aggression of the dynamic curve injection\n\n";
        std::cout << "--- SPEEDS & TIMERS ---\n";
        std::cout << "  splin    : Max linear speed\n";
        std::cout << "  spang    : Max angular speed\n";
        std::cout << "  arutime  : Marker loss timeout (sec)\n";
        std::cout << "  waittime : Interruption delay (sec)\n";
        std::cout << "  pkgtime  : Package abandon timeout (sec)\n";
        std::cout << "  depdelay : Safety departure countdown (sec)\n";
    } else {
        std::cout << "\033[1;31m[ERROR] Unknown help argument.\033[0m\n";
    }
}

/*
 * Function cmdTaxi
 * Dispatches a destination via the /cmd_taxi topic
 */
void cmdTaxi(const std::string& arg, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr list_pub, rclcpp::Logger logger) {
    if (arg.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing argument. Use 'taxi list' or 'taxi <number>'.\033[0m\n";
        return;
    }

    if (arg == "list") {
        // Send the boolean trigger to the Mover!
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        std::cout << "\033[1;33mRequesting database from Mover...\033[0m\n";
        return;
    } 
    
    // Safety check: If they try to send a number before the database is synced
    if (valid_ids.empty()) {
        std::cout << "\033[1;31m[ERROR] Database empty. Requesting sync. Please wait a second and type your command again.\033[0m\n";
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        return;
    }

    // Normal Dispatch Logic
    try {
        int target_id = std::stoi(arg);
        if (std::find(valid_ids.begin(), valid_ids.end(), target_id) != valid_ids.end()) {
            auto msg = std_msgs::msg::Int32();
            msg.data = target_id;
            pub->publish(msg);
            RCLCPP_INFO(logger, "\033[1;32mDispatched ID %d to the network.\033[0m", target_id);
        } else {
            std::cout << "\033[1;31m[ERROR] ID " << target_id << " is not in the active database.\033[0m\n";
        }
    } catch (...) {
        std::cout << "\033[1;31m[ERROR] Invalid argument. Use 'taxi list' or 'taxi <number>'.\033[0m\n";
    }
}

/*
 * Function cmdPkg
 * Broadcasts boolean overrides to the Mover network
 */
void cmdPkg(const std::string& arg, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
    if (arg == "true" || arg == "false") {
        // Update local status and push to network
        current_pkg_status = (arg == "true");
        auto msg = std_msgs::msg::Bool();
        msg.data = current_pkg_status;
        pub->publish(msg);
        std::cout << "Package status forced to: " << (current_pkg_status ? "TRUE" : "FALSE") << "\n";

    } else if (arg == "status") {
        std::cout << "  Package Taken : " << (current_pkg_status ? "TRUE" : "FALSE") << "\n";
        std::cout << "  Robot Position: " << current_robot_pos << "\n";

    } else {
        std::cout << "\033[1;31m[ERROR] Invalid argument. Use 'pkg true', 'pkg false', or 'pkg status'.\033[0m\n";
    }
}

/*
 * Function cmdParam
 * Requests a live, in-memory variable override inside the running robot, 
 * or fetches the current values from the ROS 2 parameter server.
 */
void cmdParam(const std::string& args, std::shared_ptr<rclcpp::AsyncParametersClient> param_client) {
    // Verify arguments exist
    if (args.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing arguments. Use 'param list' or 'param <name> <value>'.\033[0m\n";
        return;
    }

    // Parse the argument string into name and value
    std::istringstream iss(args);
    std::string p_name;
    iss >> p_name;

    // --- NEW: List all current parameters ---
    if (p_name == "list") {
        std::vector<std::string> param_names = {
            "xpark", "ypark", "unce", "hyst", 
            "angpark", "angunce", "anghyst", 
            "kpy", "kpx", "kperrang", "kpdynang", 
            "splin", "spang", 
            "arutime", "waittime", "pkgtime", "depdelay"
        };
        
        // Request the values from the network asynchronously
        auto future_result = param_client->get_parameters(param_names);
        
        // Wait up to 2 seconds for the robot to respond before timing out
        if (future_result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            std::cout << "\033[1;31m[ERROR] Timeout waiting for Mover node. Is it running?\033[0m\n";
            return;
        }

        auto result = future_result.get();
        std::cout << "\n\033[1;36m--- CURRENT PARAMETER VALUES ---\033[0m\n";
        for (const auto& param : result) {
            std::cout << "  " << param.get_name();
            // Pad the string to neatly align the colons in the terminal
            int pad = 10 - param.get_name().length();
            if (pad > 0) std::cout << std::string(pad, ' ');
            std::cout << " : " << param.as_double() << "\n";
        }
        std::cout << "\033[1;36m--------------------------------\033[0m\n";
        return;
    }

    // --- Default: Set a new parameter ---
    std::string p_val_str;
    iss >> p_val_str;

    if (p_val_str.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing value. Use 'param <name> <value>'.\033[0m\n";
        return;
    }

    try {
        double p_val = std::stod(p_val_str);
        // Inject parameter onto the ROS 2 parameter server
        param_client->set_parameters({rclcpp::Parameter(p_name, p_val)});
        std::cout << "\033[1;32m[SYSTEM] Network request sent to update '" << p_name << "' to " << p_val << ".\033[0m\n";
    } catch (...) {
        std::cout << "\033[1;31m[ERROR] Value must be a number.\033[0m\n";
    }
}

/*
 * Main
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    signal(SIGINT, sigintHandler);
    
    auto node = rclcpp::Node::make_shared("dispatcher_node");
    
    auto taxi_pub = node->create_publisher<std_msgs::msg::Int32>("/cmd_taxi", 10);
    auto pkg_pub = node->create_publisher<std_msgs::msg::Bool>("/pkg_status", 10);
    
    // --- The New Trigger Publisher ---
    auto db_req_pub = node->create_publisher<std_msgs::msg::Bool>("/req_db", 10);
    
    // Asynchronous Param editor referencing the exact node name declared in mover.cpp
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "aruco_follower_node");
    
    // --- The Network Callback (Prints the list when it arrives) ---
    auto db_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/active_db", 10,
        [](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            valid_ids = msg->data;
            std::cout << "\n\033[1;32m[SYSTEM] Available Destinations: [ ";
            for (int id : valid_ids) std::cout << id << " ";
            std::cout << "]\033[0m\n[DISPATCHER] > " << std::flush;
        }
    );

    // --- FIX 1: Run the ROS 2 node in a background thread so the UI doesn't block the network ---
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    RCLCPP_INFO(node->get_logger(), "\n"
        "=========================================\n"
        " Dynamic Terminal Input Node v1.0\n"
        "-----------------------------------------\n"
        " Osaka Institute of Technology\n"
        " Department of Real World Computing\n"
        " OIT ArUco Delivery Robot\n"
        " Developer : Marcos FERRANDO ESPAÑA\n"
        " Supervisor: Prof. Kenzaburo MIYAWAKI\n"
        "-----------------------------------------\n"
        " Publishing to : /cmd_taxi\n"
        " Message type  : std_msgs/msg/Int32\n"
        "-----------------------------------------\n"
        " Enter the ID of an ArUco available in the database and press ENTER to publish the command.\n"
        "=========================================\n"
        " Type 'help' for available commands.\n"
        " Waiting for user input...\n");

    rl_catch_signals = 0;
    
    while (rclcpp::ok()) {
        // --- FIX 2 & 3: readline automatically handles ANSI codes, arrow keys, and Ctrl+C crashes ---
        char* input = readline("\n[DISPATCHER] > ");
        
        // Safely catch Ctrl+C or Ctrl+D (EOF) to prevent infinite spam
        if (!input) {
            sigintHandler(0); 
        }

        std::string line(input);
        
        // Add the typed command to the up-arrow history
        if (!line.empty()) {
            add_history(input); 
        }
        
        // Prevent memory leaks from readline allocation
        free(input); 

        if (line.empty()) continue;

        // Parse user input
        std::istringstream iss(line);
        std::string command, argument;
        iss >> command;
        
        // Grab everything after the command to feed both 'name' and 'value' directly into cmdParam
        std::getline(iss >> std::ws, argument);

        // Route to the appropriate command function
        if (command == "help") {
            cmdHelp(argument);
        } else if (command == "taxi") {
            cmdTaxi(argument, taxi_pub, db_req_pub, node->get_logger());
        } else if (command == "pkg") {
            cmdPkg(argument, pkg_pub);
        } else if (command == "param") {
            cmdParam(argument, param_client);
        } else {
            std::cout << "\033[1;31m[ERROR] Unknown command. Type 'help' for a list of commands.\033[0m\n";
        }
    }

    rclcpp::shutdown();
    
    // Safely collapse the background network thread before exiting
    spin_thread.join(); 
    return 0;
}