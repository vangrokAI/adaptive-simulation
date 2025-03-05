#include "adaptive_simulation/ft_sensor.hpp"

#include <random>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace adaptive_simulation {

FTSensor::FTSensor(const rclcpp::NodeOptions& options)
: rclcpp::Node("ft_sensor_node", options) {
    // Declare and get parameters
    this->declare_parameter("sensor_frame", "ft_sensor_frame");
    this->declare_parameter("robot_base_frame", "world");
    this->declare_parameter("end_effector_frame", "lbr_link_7");
    this->declare_parameter("use_sim_time", true);
    this->declare_parameter("ceiling_mounted", true);
    this->declare_parameter("simulated_sensor", true);
    this->declare_parameter("publish_rate", 500.0); // Hz
    this->declare_parameter("sensor_noise_stddev", 0.1); // N or Nm
    this->declare_parameter("sensor_offset_x", 0.0); // m
    this->declare_parameter("sensor_offset_y", 0.0); // m
    this->declare_parameter("sensor_offset_z", 0.05); // m
    
    // ATI Axia80-M20 sensor specifications
    this->declare_parameter("max_force_x", 500.0); // N
    this->declare_parameter("max_force_y", 500.0); // N
    this->declare_parameter("max_force_z", 900.0); // N
    this->declare_parameter("max_torque_x", 20.0); // Nm
    this->declare_parameter("max_torque_y", 20.0); // Nm
    this->declare_parameter("max_torque_z", 20.0); // Nm
    this->declare_parameter("resolution_force", 0.25); // N
    this->declare_parameter("resolution_torque", 0.01); // Nm
    
    // Get parameter values
    sensor_frame_ = this->get_parameter("sensor_frame").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    end_effector_frame_ = this->get_parameter("end_effector_frame").as_string();
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    ceiling_mounted_ = this->get_parameter("ceiling_mounted").as_bool();
    simulated_sensor_ = this->get_parameter("simulated_sensor").as_bool();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    sensor_noise_stddev_ = this->get_parameter("sensor_noise_stddev").as_double();
    
    // Get sensor offset parameters
    sensor_offset_[0] = this->get_parameter("sensor_offset_x").as_double();
    sensor_offset_[1] = this->get_parameter("sensor_offset_y").as_double();
    sensor_offset_[2] = this->get_parameter("sensor_offset_z").as_double();
    
    // Get sensor specifications
    max_force_x_ = this->get_parameter("max_force_x").as_double();
    max_force_y_ = this->get_parameter("max_force_y").as_double();
    max_force_z_ = this->get_parameter("max_force_z").as_double();
    max_torque_x_ = this->get_parameter("max_torque_x").as_double();
    max_torque_y_ = this->get_parameter("max_torque_y").as_double();
    max_torque_z_ = this->get_parameter("max_torque_z").as_double();
    resolution_force_ = this->get_parameter("resolution_force").as_double();
    resolution_torque_ = this->get_parameter("resolution_torque").as_double();
    
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize publishers and subscribers
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "ft_sensor/wrench", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "ft_sensor/visualization", 10);
    
    if (simulated_sensor_) {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/lbr/joint_states", 10,
            std::bind(&FTSensor::jointStateCallback, this, std::placeholders::_1));
    }
    
    // Initialize sensor state
    current_wrench_.header.frame_id = sensor_frame_;
    current_wrench_.wrench.force.x = 0.0;
    current_wrench_.wrench.force.y = 0.0;
    current_wrench_.wrench.force.z = 0.0;
    current_wrench_.wrench.torque.x = 0.0;
    current_wrench_.wrench.torque.y = 0.0;
    current_wrench_.wrench.torque.z = 0.0;
    sensor_contact_detected_ = false;
    
    // Set up timer for publishing
    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        publish_period, std::bind(&FTSensor::publishTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), 
        "FT Sensor initialized (simulated: %s, ceiling_mounted: %s)",
        simulated_sensor_ ? "true" : "false",
        ceiling_mounted_ ? "true" : "false");
}

void FTSensor::publishTimerCallback() {
    geometry_msgs::msg::WrenchStamped wrench;
    
    if (simulated_sensor_) {
        // Generate simulated data
        wrench = simulateFTData();
    } else {
        // In real hardware mode, we'd use the data from the RDT interface
        // For now, just use the current wrench
        wrench = current_wrench_;
    }
    
    // Add realistic noise to the data
    addSensorNoise(wrench);
    
    // Apply calibration (important for real sensor data)
    wrench = calibrateWrench(wrench);
    
    // Transform to the correct reference frame if needed
    wrench = transformWrench(wrench);
    
    // Update current state
    current_wrench_ = wrench;
    
    // Publish the wrench data
    wrench_pub_->publish(wrench);
    
    // Publish visualization markers
    publishVisualization(wrench);
}

void FTSensor::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // This is where we would use joint state information to improve the simulation
    // For now, this is just a placeholder
}

geometry_msgs::msg::WrenchStamped FTSensor::simulateFTData() {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = this->now();
    wrench.header.frame_id = sensor_frame_;
    
    // Implement simple simulation logic here
    // In a real implementation, this would check for collisions with obstacles
    // and calculate appropriate forces based on penetration depth, material properties, etc.
    
    // For demonstration purposes, just generate some small forces
    // to simulate contact with environment
    if (sensor_contact_detected_) {
        wrench.wrench.force.z = ceiling_mounted_ ? 10.0 : -10.0; // 10N in Z direction (adjusted for ceiling mounting)
        wrench.wrench.torque.x = 0.5; // 0.5Nm around X axis
    } else {
        // No contact, just gravity and noise
        wrench.wrench.force.z = ceiling_mounted_ ? 1.0 : -1.0; // Tool weight
    }
    
    return wrench;
}

void FTSensor::addSensorNoise(geometry_msgs::msg::WrenchStamped& wrench) {
    // Create random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> force_noise(0.0, sensor_noise_stddev_);
    static std::normal_distribution<> torque_noise(0.0, sensor_noise_stddev_ * 0.1); // Less noise on torque
    
    // Add noise to each component
    wrench.wrench.force.x += force_noise(gen);
    wrench.wrench.force.y += force_noise(gen);
    wrench.wrench.force.z += force_noise(gen);
    wrench.wrench.torque.x += torque_noise(gen);
    wrench.wrench.torque.y += torque_noise(gen);
    wrench.wrench.torque.z += torque_noise(gen);
    
    // Quantize to sensor resolution (simulating real sensor behavior)
    wrench.wrench.force.x = std::round(wrench.wrench.force.x / resolution_force_) * resolution_force_;
    wrench.wrench.force.y = std::round(wrench.wrench.force.y / resolution_force_) * resolution_force_;
    wrench.wrench.force.z = std::round(wrench.wrench.force.z / resolution_force_) * resolution_force_;
    wrench.wrench.torque.x = std::round(wrench.wrench.torque.x / resolution_torque_) * resolution_torque_;
    wrench.wrench.torque.y = std::round(wrench.wrench.torque.y / resolution_torque_) * resolution_torque_;
    wrench.wrench.torque.z = std::round(wrench.wrench.torque.z / resolution_torque_) * resolution_torque_;
}

geometry_msgs::msg::WrenchStamped FTSensor::transformWrench(const geometry_msgs::msg::WrenchStamped& raw_wrench) {
    // This function would transform the wrench from the sensor frame to the desired output frame
    // For now, just return the input wrench
    return raw_wrench;
}

geometry_msgs::msg::WrenchStamped FTSensor::calibrateWrench(const geometry_msgs::msg::WrenchStamped& raw_wrench) {
    // Apply calibration to raw sensor data
    // For simulated data, this is not strictly necessary but included for completeness
    // In real hardware, this would apply sensor calibration matrix and bias correction
    return raw_wrench;
}

void FTSensor::publishVisualization(const geometry_msgs::msg::WrenchStamped& wrench) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Sensor body visualization
    visualization_msgs::msg::Marker sensor_marker;
    sensor_marker.header.frame_id = sensor_frame_;
    sensor_marker.header.stamp = this->now();
    sensor_marker.ns = "ft_sensor";
    sensor_marker.id = 0;
    sensor_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    sensor_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // ATI Axia80 dimensions (cylinder approximation)
    sensor_marker.scale.x = 0.08; // diameter
    sensor_marker.scale.y = 0.08; // diameter
    sensor_marker.scale.z = 0.034; // height
    
    // ATI color (blue-ish)
    sensor_marker.color.r = 0.1;
    sensor_marker.color.g = 0.3;
    sensor_marker.color.b = 0.7;
    sensor_marker.color.a = 0.9;
    
    marker_array.markers.push_back(sensor_marker);
    
    // Force vector visualization
    visualization_msgs::msg::Marker force_marker;
    force_marker.header.frame_id = sensor_frame_;
    force_marker.header.stamp = this->now();
    force_marker.ns = "ft_sensor";
    force_marker.id = 1;
    force_marker.type = visualization_msgs::msg::Marker::ARROW;
    force_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Arrow starts at sensor origin
    force_marker.points.resize(2);
    force_marker.points[0].x = 0.0;
    force_marker.points[0].y = 0.0;
    force_marker.points[0].z = 0.0;
    
    // Arrow ends in direction of force, scaled by magnitude
    double force_mag = std::sqrt(
        wrench.wrench.force.x * wrench.wrench.force.x +
        wrench.wrench.force.y * wrench.wrench.force.y +
        wrench.wrench.force.z * wrench.wrench.force.z);
    
    double scale_factor = 0.01; // 1cm per 1N
    if (force_mag > 1.0) {
        force_marker.points[1].x = wrench.wrench.force.x * scale_factor;
        force_marker.points[1].y = wrench.wrench.force.y * scale_factor;
        force_marker.points[1].z = wrench.wrench.force.z * scale_factor;
        
        // Force arrow styling
        force_marker.scale.x = 0.01; // shaft diameter
        force_marker.scale.y = 0.02; // head diameter
        force_marker.scale.z = 0.03; // head length
        
        // Red for force
        force_marker.color.r = 1.0;
        force_marker.color.g = 0.0;
        force_marker.color.b = 0.0;
        force_marker.color.a = 1.0;
        
        marker_array.markers.push_back(force_marker);
    }
    
    // Torque vector visualization (similar to force)
    visualization_msgs::msg::Marker torque_marker;
    torque_marker.header.frame_id = sensor_frame_;
    torque_marker.header.stamp = this->now();
    torque_marker.ns = "ft_sensor";
    torque_marker.id = 2;
    torque_marker.type = visualization_msgs::msg::Marker::ARROW;
    torque_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Arrow starts at sensor origin
    torque_marker.points.resize(2);
    torque_marker.points[0].x = 0.0;
    torque_marker.points[0].y = 0.0;
    torque_marker.points[0].z = 0.0;
    
    // Arrow ends in direction of torque, scaled by magnitude
    double torque_mag = std::sqrt(
        wrench.wrench.torque.x * wrench.wrench.torque.x +
        wrench.wrench.torque.y * wrench.wrench.torque.y +
        wrench.wrench.torque.z * wrench.wrench.torque.z);
    
    double torque_scale_factor = 0.1; // 10cm per 1Nm
    if (torque_mag > 0.1) {
        torque_marker.points[1].x = wrench.wrench.torque.x * torque_scale_factor;
        torque_marker.points[1].y = wrench.wrench.torque.y * torque_scale_factor;
        torque_marker.points[1].z = wrench.wrench.torque.z * torque_scale_factor;
        
        // Torque arrow styling
        torque_marker.scale.x = 0.01; // shaft diameter
        torque_marker.scale.y = 0.02; // head diameter
        torque_marker.scale.z = 0.03; // head length
        
        // Blue for torque
        torque_marker.color.r = 0.0;
        torque_marker.color.g = 0.0;
        torque_marker.color.b = 1.0;
        torque_marker.color.a = 1.0;
        
        marker_array.markers.push_back(torque_marker);
    }
    
    // Publish all markers
    marker_pub_->publish(marker_array);
}

} // namespace adaptive_simulation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(adaptive_simulation::FTSensor)
