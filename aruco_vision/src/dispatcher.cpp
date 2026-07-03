#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
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
std::string latest_telemetry = "";
std::string latest_db_str = "";

// --- POSIX SIGNAL HANDLER ---
void sigintHandler(int signum) {
    std::cout << "\n\033[1;33m[SYSTEM] Shutting down Dispatcher...\033[0m\n";
    rclcpp::shutdown();
    exit(signum); 
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
        std::cout << "  status: View live robot telemetry\n";
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
        std::cout << " param <name> <value> : Sets the parameter to the value\n";
        std::cout << " param list           : Shows current values\n\n";
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
    } else if (arg == "status") {
        std::cout << "Objective : Displays the live, real-time telemetry from the robot's control loop.\n";
        std::cout << "  status       : Displays global state and math calculations\n";
        std::cout << "  status robot : Displays only the physical state and target info\n";
        std::cout << "  status math  : Displays only the PID errors and angles\n";
        std::cout << "  status db    : Displays the fully formatted route database\n";
        std::cout << "  status queue : Displays the current destinations waiting in the FIFO queue\n";
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
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        std::cout << "Requesting database from Mover...\n";
        return;
    } 
    
    if (valid_ids.empty()) {
        std::cout << "\033[1;31m[ERROR] Database empty. Requesting sync. Please wait a second and type your command again.\033[0m\n";
        auto list_msg = std_msgs::msg::Bool();
        list_msg.data = true;
        list_pub->publish(list_msg);
        return;
    }

    try {
        int target_id = std::stoi(arg);
        if (std::find(valid_ids.begin(), valid_ids.end(), target_id) != valid_ids.end()) {
            auto msg = std_msgs::msg::Int32();
            msg.data = target_id;
            pub->publish(msg);
            RCLCPP_INFO(logger, "Dispatched ID %d to the network.", target_id);
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
    if (args.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing arguments. Use 'param list' or 'param <name> <value>'.\033[0m\n";
        return;
    }

    std::istringstream iss(args);
    std::string p_name;
    iss >> p_name;

    if (p_name == "list") {
        std::vector<std::string> param_names = {
            "xpark", "ypark", "unce", "hyst", 
            "angpark", "angunce", "anghyst", 
            "kpy", "kpx", "kperrang", "kpdynang", 
            "splin", "spang", 
            "arutime", "waittime", "pkgtime", "depdelay"
        };
        
        auto future_result = param_client->get_parameters(param_names);
        
        if (future_result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            std::cout << "\033[1;31m[ERROR] Timeout waiting for Mover node. Is it running?\033[0m\n";
            return;
        }

        auto result = future_result.get();
        std::cout << "\n--- CURRENT PARAMETER VALUES ---\n";
        for (const auto& param : result) {
            std::cout << "  " << param.get_name();
            int pad = 10 - param.get_name().length();
            if (pad > 0) std::cout << std::string(pad, ' ');
            std::cout << " : " << param.as_double() << "\n";
        }
        std::cout << "--------------------------------\n";
        return;
    }

    std::string p_val_str;
    iss >> p_val_str;

    if (p_val_str.empty()) {
        std::cout << "\033[1;31m[ERROR] Missing value. Use 'param <name> <value>'.\033[0m\n";
        return;
    }

    try {
        double p_val = std::stod(p_val_str);
        param_client->set_parameters({rclcpp::Parameter(p_name, p_val)});
        std::cout << "[SYSTEM] Network request sent to update '" << p_name << "' to " << p_val << ".\n";
    } catch (...) {
        std::cout << "\033[1;31m[ERROR] Value must be a number.\033[0m\n";
    }
}

/*
 * Function cmdStatus
 * Parses the live telemetry string and formats it based on the requested detail level
 */
void cmdStatus(const std::string& arg) {
    if (arg == "db") {
        std::cout << "\n--- ROUTE DATABASE ---\n";
        if (latest_db_str.empty()) {
            std::cout << "  Database empty. Type 'taxi list' to sync.\n";
        } else {
            std::cout << latest_db_str;
        }
        std::cout << "----------------------\n";
        return;
    }

    if (latest_telemetry.empty()) {
        std::cout << "\033[1;31m[ERROR] No telemetry received. Is Mover running?\033[0m\n";
        return;
    }
    
    std::vector<std::string> t;
    std::stringstream ss(latest_telemetry);
    std::string token;
    while (std::getline(ss, token, ',')) t.push_back(token);
    
    if (t.size() < 14) return;

    int rs = std::stoi(t[0]);
    const char* state_str[] = {"IDLE", "MOVE", "PARK", "SEARCH", "WAIT"};
    std::string c_state = (rs >= 0 && rs <= 4) ? state_str[rs] : "UNKNOWN";

    if (arg.empty()) {
        std::cout << "\n[GLOBAL STATUS] " 
                  << "State: " << c_state 
                  << " | Target: " << t[1] 
                  << " | Vis: " << (t[3] == "1" ? "Y" : "N") 
                  << " | Queue: " << t[11] 
                  << " | Lin: " << t[9] 
                  << " | Ang: " << t[10] << "\n";
    }
    else if (arg == "robot") {
        std::cout << "\n--- ROBOT STATE ---\n";
        std::cout << "  Current State : " << c_state << "\n";
        std::cout << "  Target ID     : " << t[1] << "\n";
        std::cout << "  Is Destination: " << (t[2] == "1" ? "TRUE" : "FALSE") << "\n";
        std::cout << "  ArUco Visible : " << (t[3] == "1" ? "YES" : "NO") << "\n";
        std::cout << "  Pending Queue : " << t[11] << " destinations\n";
        std::cout << "  Package Status: " << (t[12] == "1" ? "TAKEN" : "WAITING") << "\n";
        std::cout << "  Command Vel   : [Lin: " << t[9] << ", Ang: " << t[10] << "]\n";
        std::cout << "-------------------\n";
    }
    else if (arg == "math") {
        std::cout << "\n--- MATH & ALIGNMENT ---\n";
        std::cout << "  Error X       : " << t[4] << " px\n";
        std::cout << "  Error Y       : " << t[5] << " px\n";
        std::cout << "  Raw Angle     : " << t[6] << " rad\n";
        std::cout << "  Dynamic Angle : " << t[7] << " rad\n";
        std::cout << "  Error Angle   : " << t[8] << " rad\n";
        std::cout << "------------------------\n";
    }
    else if (arg == "queue") {
        std::cout << "\n--- PENDING QUEUE ---\n";
        std::cout << "  " << t[13] << "\n";
        std::cout << "---------------------\n";
    }
    else {
        std::cout << "\033[1;31m[ERROR] Unknown argument. Use 'status', 'status robot', 'status math', 'status db', or 'status queue'.\033[0m\n";
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
    auto db_req_pub = node->create_publisher<std_msgs::msg::Bool>("/req_db", 10);
    
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "aruco_follower_node");
    
    auto db_sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/active_db", 10,
        [](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            valid_ids = msg->data;
            std::cout << "\n[SYSTEM] Available Destinations: [ ";
            for (int id : valid_ids) std::cout << id << " ";
            std::cout << "]\n[DISPATCHER] > " << std::flush;
        }
    );

    auto db_str_sub = node->create_subscription<std_msgs::msg::String>(
        "/active_db_str", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            latest_db_str = msg->data;
        }
    );

    auto tel_sub = node->create_subscription<std_msgs::msg::String>(
        "/telem", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            latest_telemetry = msg->data;
        }
    );

    // Web Server Integration
    auto mission_sub = node->create_subscription<std_msgs::msg::String>(
        "/mission", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            std::cout << "\n[WEB SERVER] Received target ID: " << msg->data << " from HTML interface.\n[DISPATCHER] > " << std::flush;
        }
    );

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
        char* input = readline("\n[DISPATCHER] > ");
        
        if (!input) {
            sigintHandler(0); 
        }

        std::string line(input);
        
        if (!line.empty()) {
            add_history(input); 
        }
        
        free(input); 

        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string command, argument;
        iss >> command;
        
        std::getline(iss >> std::ws, argument);

        if (command == "help") {
            cmdHelp(argument);
        } else if (command == "taxi") {
            cmdTaxi(argument, taxi_pub, db_req_pub, node->get_logger());
        } else if (command == "pkg") {
            cmdPkg(argument, pkg_pub);
        } else if (command == "param") {
            cmdParam(argument, param_client);
        } else if (command == "status") {
            cmdStatus(argument);
        } else {
            std::cout << "\033[1;31m[ERROR] Unknown command. Type 'help' for a list of commands.\033[0m\n";
        }
    }

    rclcpp::shutdown();
    spin_thread.join(); 
    return 0;
}