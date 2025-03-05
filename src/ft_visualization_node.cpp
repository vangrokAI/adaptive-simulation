#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class FTVisualizationNode : public rclcpp::Node {
public:
    FTVisualizationNode() : Node("ft_visualization_node") {
        // Declare parameters
        this->declare_parameter("use_sim_time", true);
        this->declare_parameter("sensor_frame", "ati_axia80_measurement_link");
        this->declare_parameter("force_arrow_scale", 0.01);  // 1cm per 1N
        this->declare_parameter("torque_arrow_scale", 0.02); // 2cm per 1Nm
        this->declare_parameter("force_topic", "/ft_sensor/wrench");
        this->declare_parameter("ceiling_mounted", true);
        
        // Get parameters
        use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
        sensor_frame_ = this->get_parameter("sensor_frame").as_string();
        force_arrow_scale_ = this->get_parameter("force_arrow_scale").as_double();
        torque_arrow_scale_ = this->get_parameter("torque_arrow_scale").as_double();
        force_topic_ = this->get_parameter("force_topic").as_string();
        ceiling_mounted_ = this->get_parameter("ceiling_mounted").as_bool();
        
        // Setup TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ft_sensor/visualization", 10);
        
        // Create subscribers
        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            force_topic_, 10, 
            std::bind(&FTVisualizationNode::wrenchCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "F/T Visualization node initialized");
    }

private:
    // Parameters
    bool use_sim_time_;
    std::string sensor_frame_;
    double force_arrow_scale_;
    double torque_arrow_scale_;
    std::string force_topic_;
    bool ceiling_mounted_;
    
    // Publishers and subscribers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Create ATI Axia80 sensor body visualization
        visualization_msgs::msg::Marker sensor_body;
        sensor_body.header.frame_id = msg->header.frame_id;
        sensor_body.header.stamp = msg->header.stamp;
        sensor_body.ns = "ft_visualization";
        sensor_body.id = 0;
        sensor_body.type = visualization_msgs::msg::Marker::CYLINDER;
        sensor_body.action = visualization_msgs::msg::Marker::ADD;
        
        // ATI Axia80 dimensions (cylinder approximation)
        sensor_body.scale.x = 0.08; // diameter (m)
        sensor_body.scale.y = 0.08; // diameter (m)
        sensor_body.scale.z = 0.034; // height (m)
        
        // ATI blue color
        sensor_body.color.r = 0.1;
        sensor_body.color.g = 0.3;
        sensor_body.color.b = 0.7;
        sensor_body.color.a = 0.9;
        
        // Position (identity, as we're using the sensor frame)
        sensor_body.pose.position.x = 0.0;
        sensor_body.pose.position.y = 0.0;
        sensor_body.pose.position.z = 0.0;
        sensor_body.pose.orientation.x = 0.0;
        sensor_body.pose.orientation.y = 0.0;
        sensor_body.pose.orientation.z = 0.0;
        sensor_body.pose.orientation.w = 1.0;
        
        marker_array.markers.push_back(sensor_body);
        
        // Visualize force as an arrow
        double force_magnitude = std::sqrt(
            msg->wrench.force.x * msg->wrench.force.x +
            msg->wrench.force.y * msg->wrench.force.y +
            msg->wrench.force.z * msg->wrench.force.z);
        
        if (force_magnitude > 1.0) { // Only show arrows for forces > 1N
            visualization_msgs::msg::Marker force_arrow;
            force_arrow.header.frame_id = msg->header.frame_id;
            force_arrow.header.stamp = msg->header.stamp;
            force_arrow.ns = "ft_visualization";
            force_arrow.id = 1;
            force_arrow.type = visualization_msgs::msg::Marker::ARROW;
            force_arrow.action = visualization_msgs::msg::Marker::ADD;
            
            // Start at sensor center
            force_arrow.points.resize(2);
            force_arrow.points[0].x = 0.0;
            force_arrow.points[0].y = 0.0;
            force_arrow.points[0].z = 0.0;
            
            // End in direction of force, scaled by magnitude and scale factor
            force_arrow.points[1].x = msg->wrench.force.x * force_arrow_scale_;
            force_arrow.points[1].y = msg->wrench.force.y * force_arrow_scale_;
            force_arrow.points[1].z = msg->wrench.force.z * force_arrow_scale_;
            
            // Arrow styling
            force_arrow.scale.x = 0.01; // shaft diameter
            force_arrow.scale.y = 0.02; // head diameter
            force_arrow.scale.z = 0.03; // head length
            
            // Red for force
            force_arrow.color.r = 1.0;
            force_arrow.color.g = 0.0;
            force_arrow.color.b = 0.0;
            force_arrow.color.a = 1.0;
            
            marker_array.markers.push_back(force_arrow);
            
            // Add force text label
            visualization_msgs::msg::Marker force_text;
            force_text.header.frame_id = msg->header.frame_id;
            force_text.header.stamp = msg->header.stamp;
            force_text.ns = "ft_visualization";
            force_text.id = 3;
            force_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            force_text.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text at end of force arrow
            force_text.pose.position = force_arrow.points[1];
            force_text.pose.position.z += 0.05; // Offset text above arrow
            
            // Text content: force magnitude
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << force_magnitude << " N";
            force_text.text = ss.str();
            
            // Text styling
            force_text.scale.z = 0.04; // text height
            
            // White text
            force_text.color.r = 1.0;
            force_text.color.g = 1.0;
            force_text.color.b = 1.0;
            force_text.color.a = 1.0;
            
            marker_array.markers.push_back(force_text);
        }
        
        // Visualize torque as an arrow
        double torque_magnitude = std::sqrt(
            msg->wrench.torque.x * msg->wrench.torque.x +
            msg->wrench.torque.y * msg->wrench.torque.y +
            msg->wrench.torque.z * msg->wrench.torque.z);
        
        if (torque_magnitude > 0.2) { // Only show arrows for torques > 0.2Nm
            visualization_msgs::msg::Marker torque_arrow;
            torque_arrow.header.frame_id = msg->header.frame_id;
            torque_arrow.header.stamp = msg->header.stamp;
            torque_arrow.ns = "ft_visualization";
            torque_arrow.id = 2;
            torque_arrow.type = visualization_msgs::msg::Marker::ARROW;
            torque_arrow.action = visualization_msgs::msg::Marker::ADD;
            
            // Start at sensor center
            torque_arrow.points.resize(2);
            torque_arrow.points[0].x = 0.0;
            torque_arrow.points[0].y = 0.0;
            torque_arrow.points[0].z = 0.0;
            
            // End in direction of torque, scaled by magnitude and scale factor
            torque_arrow.points[1].x = msg->wrench.torque.x * torque_arrow_scale_;
            torque_arrow.points[1].y = msg->wrench.torque.y * torque_arrow_scale_;
            torque_arrow.points[1].z = msg->wrench.torque.z * torque_arrow_scale_;
            
            // Arrow styling
            torque_arrow.scale.x = 0.01; // shaft diameter
            torque_arrow.scale.y = 0.02; // head diameter
            torque_arrow.scale.z = 0.03; // head length
            
            // Blue for torque
            torque_arrow.color.r = 0.0;
            torque_arrow.color.g = 0.3;
            torque_arrow.color.b = 1.0;
            torque_arrow.color.a = 1.0;
            
            marker_array.markers.push_back(torque_arrow);
            
            // Add torque text label
            visualization_msgs::msg::Marker torque_text;
            torque_text.header.frame_id = msg->header.frame_id;
            torque_text.header.stamp = msg->header.stamp;
            torque_text.ns = "ft_visualization";
            torque_text.id = 4;
            torque_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            torque_text.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text at end of torque arrow
            torque_text.pose.position = torque_arrow.points[1];
            torque_text.pose.position.z += 0.05; // Offset text above arrow
            
            // Text content: torque magnitude
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << torque_magnitude << " Nm";
            torque_text.text = ss.str();
            
            // Text styling
            torque_text.scale.z = 0.04; // text height
            
            // Light blue text
            torque_text.color.r = 0.6;
            torque_text.color.g = 0.8;
            torque_text.color.b = 1.0;
            torque_text.color.a = 1.0;
            
            marker_array.markers.push_back(torque_text);
        }
        
        // Publish all markers
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
