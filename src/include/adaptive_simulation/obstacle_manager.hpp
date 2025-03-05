#ifndef ADAPTIVE_SIMULATION_OBSTACLE_MANAGER_HPP
#define ADAPTIVE_SIMULATION_OBSTACLE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace adaptive_simulation
{

/**
 * @struct Obstacle
 * @brief Represents a virtual obstacle in the simulation
 */
struct Obstacle {
  std::string id;                      // Unique identifier
  std::string type;                    // Box, cylinder, sphere, etc.
  geometry_msgs::msg::Pose pose;       // Pose in world frame
  std::vector<double> dimensions;      // Dimensions (type-specific)
  std::string material;                // Material properties
  double contact_stiffness;            // Stiffness for force calculation
  double friction_coefficient;         // Friction coefficient
};

/**
 * @class ObstacleManager
 * @brief Manages virtual obstacles for the surface contact simulation
 * 
 * This node handles creation, updating, and visualization of virtual obstacles
 * that the robot can interact with during the simulation. It supports loading
 * obstacle configurations from files and provides services to modify obstacles
 * during runtime.
 */
class ObstacleManager : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit ObstacleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  virtual ~ObstacleManager();

private:
  // ROS interfaces
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_config_srv_;
  
  // State variables
  std::vector<Obstacle> obstacles_;
  std::string config_file_path_;
  
  // Callbacks
  void timerCallback();
  void reloadConfigCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // Helper methods
  bool loadObstacleConfig();
  void publishObstacleMarkers();
  void broadcastObstacleTFs();
  visualization_msgs::msg::Marker createMarkerFromObstacle(const Obstacle & obstacle);
};

} // namespace adaptive_simulation

#endif // ADAPTIVE_SIMULATION_OBSTACLE_MANAGER_HPP
