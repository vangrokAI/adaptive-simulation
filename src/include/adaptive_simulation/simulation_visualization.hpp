#ifndef ADAPTIVE_SIMULATION_SIMULATION_VISUALIZATION_HPP
#define ADAPTIVE_SIMULATION_SIMULATION_VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace adaptive_simulation
{

/**
 * @struct ForceDataPoint
 * @brief Structure to store force data for visualization
 */
struct ForceDataPoint {
  double timestamp;
  double force_x;
  double force_y;
  double force_z;
  double magnitude;
};

/**
 * @class SimulationVisualization
 * @brief Provides enhanced visualization for the surface contact simulation
 * 
 * This node handles visualization of simulation data including force vectors,
 * robot trajectories, contact states, and diagnostic information. It creates
 * rich, interactive visualizations for RViz.
 */
class SimulationVisualization : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit SimulationVisualization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  virtual ~SimulationVisualization();

private:
  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr force_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_scale_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State variables
  std::string current_phase_;
  double current_velocity_scale_;
  geometry_msgs::msg::WrenchStamped current_wrench_;
  std::vector<ForceDataPoint> force_history_;
  int history_size_;
  
  // Configuration parameters
  std::string robot_base_frame_;
  std::string end_effector_frame_;
  std::string force_sensor_frame_;
  double force_arrow_scale_;
  double torque_arrow_scale_;
  double text_scale_;
  double update_frequency_;
  
  // Color parameters
  int force_color_r_;
  int force_color_g_;
  int force_color_b_;
  int torque_color_r_;
  int torque_color_g_;
  int torque_color_b_;
  
  // Callbacks
  void timerCallback();
  void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void velocityScaleCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void phaseCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // Helper methods
  void updateForceVisualization();
  void updateStatusDisplay();
  void updateTrajectoryVisualization();
  void addForceHistoryPoint(const geometry_msgs::msg::WrenchStamped & wrench);
  visualization_msgs::msg::MarkerArray createForceMarkers();
  visualization_msgs::msg::MarkerArray createStatusMarkers();
  visualization_msgs::msg::MarkerArray createTrajectoryMarkers();
};

} // namespace adaptive_simulation

#endif // ADAPTIVE_SIMULATION_SIMULATION_VISUALIZATION_HPP
