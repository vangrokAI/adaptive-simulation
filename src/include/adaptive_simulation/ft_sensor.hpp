#ifndef ADAPTIVE_SIMULATION_FT_SENSOR_HPP
#define ADAPTIVE_SIMULATION_FT_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Geometry>

namespace adaptive_simulation {

/**
 * @class FTSensor
 * @brief Simulates and handles communication with an ATI Axia80-M20 F/T sensor.
 * 
 * This class provides both simulated force/torque data in simulation mode and
 * interfaces with the real ATI Axia80-M20 sensor in hardware mode. It also
 * handles visualization of the sensor and forces/torques in RViz.
 */
class FTSensor : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the FTSensor class.
     * @param options Node options for configuration.
     */
    explicit FTSensor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
     * @brief Destructor for the FTSensor class.
     */
    ~FTSensor() override = default;

private:
    // Parameters
    std::string sensor_frame_;              ///< TF frame of the F/T sensor
    std::string robot_base_frame_;          ///< TF frame of the robot base
    std::string end_effector_frame_;        ///< TF frame of the end effector (where sensor is mounted)
    bool use_sim_time_;                     ///< Whether to use simulation time
    bool ceiling_mounted_;                  ///< Whether the robot is ceiling-mounted
    bool simulated_sensor_;                 ///< Whether to use simulated sensor data
    double publish_rate_;                   ///< Rate at which to publish sensor data
    double sensor_noise_stddev_;            ///< Standard deviation of simulated sensor noise
    Eigen::Vector3d sensor_offset_;         ///< Offset of the sensor from the end effector

    // Sensor characteristics (ATI Axia80-M20)
    double max_force_x_;                    ///< Maximum force in X direction (N)
    double max_force_y_;                    ///< Maximum force in Y direction (N)
    double max_force_z_;                    ///< Maximum force in Z direction (N)
    double max_torque_x_;                   ///< Maximum torque around X axis (Nm)
    double max_torque_y_;                   ///< Maximum torque around Y axis (Nm)
    double max_torque_z_;                   ///< Maximum torque around Z axis (Nm)
    double resolution_force_;               ///< Force resolution (N)
    double resolution_torque_;              ///< Torque resolution (Nm)
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;           ///< Publisher for wrench data
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;        ///< Publisher for visualization markers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;        ///< Subscriber to joint states
    rclcpp::TimerBase::SharedPtr timer_;                                                   ///< Timer for regular publishing
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                           ///< TF buffer for transformations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                              ///< TF listener
    
    // Current state
    geometry_msgs::msg::WrenchStamped current_wrench_;                                     ///< Current wrench measurement
    bool sensor_contact_detected_;                                                         ///< Whether contact is currently detected

    /**
     * @brief Timer callback for publishing simulated sensor data.
     */
    void publishTimerCallback();
    
    /**
     * @brief Callback for joint state messages.
     * @param msg Joint state message.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    /**
     * @brief Simulate force/torque data based on joint states and virtual obstacles.
     * @return Simulated wrench data.
     */
    geometry_msgs::msg::WrenchStamped simulateFTData();
    
    /**
     * @brief Add realistic noise to simulated sensor data.
     * @param wrench Wrench data to add noise to.
     */
    void addSensorNoise(geometry_msgs::msg::WrenchStamped& wrench);
    
    /**
     * @brief Convert raw sensor data to properly oriented wrench in robot base frame.
     * @param raw_wrench Raw wrench data from sensor.
     * @return Transformed wrench data.
     */
    geometry_msgs::msg::WrenchStamped transformWrench(const geometry_msgs::msg::WrenchStamped& raw_wrench);
    
    /**
     * @brief Apply sensor calibration to raw data.
     * @param raw_wrench Raw wrench data from sensor.
     * @return Calibrated wrench data.
     */
    geometry_msgs::msg::WrenchStamped calibrateWrench(const geometry_msgs::msg::WrenchStamped& raw_wrench);
    
    /**
     * @brief Publish visualization markers for the sensor and measured forces/torques.
     * @param wrench Current wrench measurement.
     */
    void publishVisualization(const geometry_msgs::msg::WrenchStamped& wrench);
};

} // namespace adaptive_simulation

#endif // ADAPTIVE_SIMULATION_FT_SENSOR_HPP
