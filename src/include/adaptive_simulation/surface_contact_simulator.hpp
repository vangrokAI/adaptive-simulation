#ifndef ADAPTIVE_SIMULATION_SURFACE_CONTACT_SIMULATOR_HPP
#define ADAPTIVE_SIMULATION_SURFACE_CONTACT_SIMULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

namespace adaptive_simulation
{

/**
 * @class SurfaceContactSimulator
 * @brief Simulates contact forces based on robot position and environment obstacles
 * 
 * This node continuously monitors the robot's position and calculates simulated
 * force feedback when approaching or contacting virtual obstacles. It supports
 * configurable force profiles and contact behaviors.
 */
class SurfaceContactSimulator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit SurfaceContactSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  virtual ~SurfaceContactSimulator();

private:
  // Internal enums
  enum class ContactState {
    NO_CONTACT,
    APPROACHING,
    CONTACT,
    EXCEEDED_THRESHOLD
  };

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_scale_pub_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State variables
  ContactState current_state_;
  geometry_msgs::msg::WrenchStamped current_wrench_;
  double current_velocity_scale_;
  std::vector<double> joint_positions_;
  
  // Configuration parameters
  std::string robot_base_frame_;
  std::string end_effector_frame_;
  std::string force_sensor_frame_;
  double force_threshold_;
  double approach_distance_;
  double contact_force_max_;
  double publish_frequency_;
  
  // Callbacks
  void timerCallback();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  
  // Helper methods
  void updateContactState();
  void calculateSimulatedForce();
  void publishForceAndMarkers();
  void updateVelocityScale();
  bool checkObstacleProximity(double & distance, geometry_msgs::msg::Vector3 & direction);
  void createForceMarkers(visualization_msgs::msg::MarkerArray & markers);
};

} // namespace adaptive_simulation

#endif // ADAPTIVE_SIMULATION_SURFACE_CONTACT_SIMULATOR_HPP
