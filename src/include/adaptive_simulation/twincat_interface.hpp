#ifndef ADAPTIVE_SIMULATION_TWINCAT_INTERFACE_HPP
#define ADAPTIVE_SIMULATION_TWINCAT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <memory>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace adaptive_simulation {

/**
 * @class TwincatInterface
 * @brief Interface to communicate with Beckhoff TwinCAT 3 PLC for force/torque sensor data
 * 
 * This class provides integration with TwinCAT 3 PLC systems for exchanging sensor data
 * via either EtherCAT or ADS protocol. It supports both direct communication and can also
 * use a ROS2 bridge node running on a Windows system with TwinCAT.
 */
class TwincatInterface {
public:
    /**
     * @brief Constructor for TwincatInterface
     * @param node ROS2 node to use for parameter access and logging
     */
    explicit TwincatInterface(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Destructor, cleans up network connections
     */
    ~TwincatInterface();
    
    /**
     * @brief Initialize the TwinCAT interface
     * @return True if initialization was successful
     */
    bool initialize();
    
    /**
     * @brief Start the communication interface
     * @return True if started successfully
     */
    bool start();
    
    /**
     * @brief Stop the communication interface
     */
    void stop();
    
    /**
     * @brief Send wrench data to TwinCAT system
     * @param wrench The force/torque data to send
     * @return True if data was sent successfully
     */
    bool sendWrenchData(const geometry_msgs::msg::WrenchStamped& wrench);
    
    /**
     * @brief Set the biasing state (reset force/torque to zero)
     * @param enable Whether to enable biasing
     * @return True if biasing command was sent successfully
     */
    bool setBiasing(bool enable);
    
    /**
     * @brief Check if the TwinCAT connection is active
     * @return True if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Get the last error message
     * @return Error message string
     */
    std::string getLastError() const;

private:
    // ROS2 node
    rclcpp::Node::SharedPtr node_;
    
    // Parameters
    bool use_ethercat_;
    bool use_ads_;
    std::string twincat_ip_;
    std::string lan_interface_;
    std::string ads_net_id_;
    int ads_port_;
    int ethercat_slave_id_;
    int cycletime_ms_;
    
    // Connection state
    bool is_initialized_;
    bool is_connected_;
    std::string last_error_;
    
    // ADS communication
    std::string wrench_variable_path_;
    std::string bias_variable_path_;
    
    // Communication thread
    std::thread comm_thread_;
    std::atomic<bool> keep_running_;
    std::mutex mutex_;
    
    // ROS publishers/subscribers for bridge mode
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bias_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_;
    
    // Private implementation methods
    void communicationThreadFunc();
    bool setupEtherCAT();
    bool setupADS();
    bool setupROSBridge();
    void connectionCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void updateConnectionStatus();
};

} // namespace adaptive_simulation

#endif // ADAPTIVE_SIMULATION_TWINCAT_INTERFACE_HPP
