#include "adaptive_simulation/twincat_interface.hpp"
#include <chrono>
#include <functional>

namespace adaptive_simulation {

TwincatInterface::TwincatInterface(rclcpp::Node::SharedPtr node)
    : node_(node),
      is_initialized_(false),
      is_connected_(false),
      keep_running_(false) {
    
    // Get parameters
    node_->declare_parameter("use_ethercat", false);
    node_->declare_parameter("use_ads", true);
    node_->declare_parameter("twincat_ip", "192.168.2.100");
    node_->declare_parameter("lan_interface", "eth0");
    node_->declare_parameter("ads_net_id", "192.168.2.100.1.1");
    node_->declare_parameter("ads_port", 851);
    node_->declare_parameter("ethercat_slave_id", 1);
    node_->declare_parameter("cycletime_ms", 2);
    
    use_ethercat_ = node_->get_parameter("use_ethercat").as_bool();
    use_ads_ = node_->get_parameter("use_ads").as_bool();
    twincat_ip_ = node_->get_parameter("twincat_ip").as_string();
    lan_interface_ = node_->get_parameter("lan_interface").as_string();
    ads_net_id_ = node_->get_parameter("ads_net_id").as_string();
    ads_port_ = node_->get_parameter("ads_port").as_int();
    ethercat_slave_id_ = node_->get_parameter("ethercat_slave_id").as_int();
    cycletime_ms_ = node_->get_parameter("cycletime_ms").as_int();
    
    // Variable paths for TwinCAT (default paths)
    node_->declare_parameter("wrench_variable_path", "MAIN.FTSensor.Wrench");
    node_->declare_parameter("bias_variable_path", "MAIN.FTSensor.Bias");
    
    wrench_variable_path_ = node_->get_parameter("wrench_variable_path").as_string();
    bias_variable_path_ = node_->get_parameter("bias_variable_path").as_string();
    
    RCLCPP_INFO(node_->get_logger(), "TwinCAT interface created with IP: %s", twincat_ip_.c_str());
}

TwincatInterface::~TwincatInterface() {
    stop();
}

bool TwincatInterface::initialize() {
    // If already initialized, stop first
    if (is_initialized_) {
        stop();
    }
    
    // Setup the appropriate communication method
    bool success = false;
    
    // Always start with the ROS bridge as it's the most platform-independent
    success = setupROSBridge();
    
    // If ROS bridge is not available and we want EtherCAT, try EtherCAT
    if (!success && use_ethercat_) {
        RCLCPP_INFO(node_->get_logger(), "Trying EtherCAT connection...");
        success = setupEtherCAT();
    }
    
    // If EtherCAT is not available or not configured, try ADS
    if (!success && use_ads_) {
        RCLCPP_INFO(node_->get_logger(), "Trying ADS connection...");
        success = setupADS();
    }
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "TwinCAT interface initialized successfully");
        is_initialized_ = true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize TwinCAT interface: %s", last_error_.c_str());
        is_initialized_ = false;
    }
    
    return is_initialized_;
}

bool TwincatInterface::start() {
    if (!is_initialized_) {
        last_error_ = "Cannot start: interface not initialized";
        RCLCPP_ERROR(node_->get_logger(), last_error_.c_str());
        return false;
    }
    
    if (keep_running_) {
        // Already running
        return true;
    }
    
    // Start the communication thread
    keep_running_ = true;
    comm_thread_ = std::thread(&TwincatInterface::communicationThreadFunc, this);
    
    RCLCPP_INFO(node_->get_logger(), "TwinCAT interface started");
    return true;
}

void TwincatInterface::stop() {
    if (keep_running_) {
        keep_running_ = false;
        
        if (comm_thread_.joinable()) {
            comm_thread_.join();
        }
    }
    
    is_connected_ = false;
    RCLCPP_INFO(node_->get_logger(), "TwinCAT interface stopped");
}

bool TwincatInterface::sendWrenchData(const geometry_msgs::msg::WrenchStamped& wrench) {
    if (!is_connected_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // In ROS bridge mode, publish to the wrench topic
    if (wrench_pub_) {
        wrench_pub_->publish(wrench);
        return true;
    }
    
    // Direct communication would be implemented here
    // This would be platform-specific code for ADS or EtherCAT
    
    // Since direct communication requires platform-specific libraries,
    // we'll skip the implementation here and assume it failed
    RCLCPP_WARN(node_->get_logger(), "Direct TwinCAT communication not implemented, using ROS bridge");
    return false;
}

bool TwincatInterface::setBiasing(bool enable) {
    if (!is_connected_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // In ROS bridge mode, publish to the bias topic
    if (bias_pub_) {
        std_msgs::msg::Bool msg;
        msg.data = enable;
        bias_pub_->publish(msg);
        return true;
    }
    
    // Direct communication would be implemented here
    
    RCLCPP_WARN(node_->get_logger(), "Direct TwinCAT communication not implemented, using ROS bridge");
    return false;
}

bool TwincatInterface::isConnected() const {
    return is_connected_;
}

std::string TwincatInterface::getLastError() const {
    return last_error_;
}

void TwincatInterface::communicationThreadFunc() {
    while (keep_running_) {
        // Check connection status periodically
        updateConnectionStatus();
        
        // Sleep for a short time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool TwincatInterface::setupEtherCAT() {
    // This would require platform-specific EtherCAT libraries
    // For simulation purposes, we'll just log that it's not implemented
    last_error_ = "EtherCAT communication not implemented in simulation mode";
    RCLCPP_WARN(node_->get_logger(), last_error_.c_str());
    return false;
}

bool TwincatInterface::setupADS() {
    // This would require platform-specific ADS libraries
    // For simulation purposes, we'll just log that it's not implemented
    last_error_ = "ADS communication not implemented in simulation mode";
    RCLCPP_WARN(node_->get_logger(), last_error_.c_str());
    return false;
}

bool TwincatInterface::setupROSBridge() {
    // Create publishers and subscribers for ROS bridge mode
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/twincat_bridge/wrench", 10);
    
    bias_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/twincat_bridge/bias", 10);
    
    connection_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/twincat_bridge/connected", 10,
        std::bind(&TwincatInterface::connectionCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), 
                "Set up ROS bridge mode for TwinCAT. Waiting for connection on /twincat_bridge/connected");
    
    // Try to detect if the bridge is running
    std::atomic<bool> bridge_detected(false);
    auto timer = node_->create_wall_timer(
        std::chrono::seconds(5),
        [this, &bridge_detected]() {
            if (is_connected_) {
                bridge_detected = true;
            }
        });
    
    // Wait a short time to see if we get any connection messages
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // If connected, we're good to go
    if (is_connected_) {
        RCLCPP_INFO(node_->get_logger(), "ROS bridge connected successfully");
        return true;
    }
    
    RCLCPP_WARN(node_->get_logger(), 
               "No connection detected on ROS bridge. The bridge node may not be running.");
    
    // Keep the publishers and subscribers in case the bridge connects later
    return false;
}

void TwincatInterface::connectionCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool new_state = msg->data;
    
    if (new_state != is_connected_) {
        is_connected_ = new_state;
        if (is_connected_) {
            RCLCPP_INFO(node_->get_logger(), "Connected to TwinCAT bridge");
        } else {
            RCLCPP_WARN(node_->get_logger(), "Disconnected from TwinCAT bridge");
        }
    }
}

void TwincatInterface::updateConnectionStatus() {
    // If we're using the ROS bridge, the connectionCallback handles this
    // For direct communication, we would check here
}

} // namespace adaptive_simulation
