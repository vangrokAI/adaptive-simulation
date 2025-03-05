# Adaptive Controller Architecture

## Overview

This document outlines the architecture for the Adaptive Controller implementation designed to work with the KUKA LBR IIWA robot using force/torque sensor data from the ATI Axia80-M20. The key design principle is to maintain the integrity of the standard LBR-Stack while providing adaptive behavior through non-intrusive interfaces.

## Design Philosophy

The Adaptive Controller is built on the following principles:

1. **Non-Intrusive Integration**: Never modify the core LBR-Stack
2. **Interface-Based Approach**: Use only the public interfaces provided by the LBR-Stack
3. **Separation of Concerns**: Keep force/torque sensing separate from adaptive control logic
4. **Plugin Architecture**: Support different adaptive strategies through a plugin system
5. **Configuration Flexibility**: Allow comprehensive configuration without code changes

## System Architecture

The system consists of three main components:

1. **LBR-Stack**: Unmodified standard stack (external dependency)
2. **FT-TwinCAT Bridge**: Force/torque sensor interface to the Beckhoff C6030
3. **Adaptive Controller**: Standalone ROS2 controller package

### Component Interaction

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│                 │    │                 │    │                 │
│    LBR-Stack    │◄───┤ Adaptive        │◄───┤ FT-TwinCAT     │
│    (Unmodified) │    │ Controller      │    │ Bridge          │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       ▲                      ▲
        │                       │                      │
        ▼                       │                      │
┌─────────────────┐             │               ┌─────────────┐
│                 │             │               │             │
│  Joint States   │─────────────┘               │  ATI Axia80 │
│  & Commands     │                             │  via C6030  │
│                 │                             │             │
└─────────────────┘                             └─────────────┘
```

## Implementation Details

### FT-TwinCAT Bridge

The FT-TwinCAT Bridge provides:

- ADS communication with the Beckhoff C6030
- EtherCAT configuration for the ATI Axia80-M20
- ROS2 topic publication of sensor data
- Simulation capability for development without hardware

### Adaptive Controller

The Adaptive Controller is implemented as:

1. **ROS2 Controller**: Using `controller_manager::ChainableControllerInterface`
2. **Controller Chain**: Inserted in the control chain after the `joint_trajectory_controller`
3. **Command Modification**: Adjusts the joint trajectory based on F/T feedback

```cpp
// Simplified Controller Interface
class AdaptiveController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration 
  command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration 
  state_interface_configuration() const override;
  
  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  // Force/Torque callback
  void ft_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  
private:
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  std::shared_ptr<adaptive_strategies::AdaptiveStrategyBase> strategy_;
};
```

### Plugin Architecture

The Adaptive Controller supports multiple adaptive strategies through a plugin system:

```cpp
namespace adaptive_strategies {

class AdaptiveStrategyBase
{
public:
  virtual ~AdaptiveStrategyBase() = default;
  
  virtual void configure(const rclcpp::Node::SharedPtr& node) = 0;
  
  virtual void update(
    const geometry_msgs::msg::WrenchStamped& ft_data,
    std::vector<double>& joint_commands) = 0;
};

// Example implementation
class ImpedanceStrategy : public AdaptiveStrategyBase
{
  // Impedance-based adaptation implementation
};

class ForceComplianceStrategy : public AdaptiveStrategyBase
{
  // Force compliance implementation
};

} // namespace adaptive_strategies
```

## Configuration

The Adaptive Controller is configured through ROS2 parameters:

```yaml
adaptive_controller:
  ros__parameters:
    # Controller settings
    strategy: "impedance"  # or "force_compliance", etc.
    update_rate: 1000.0    # Hz
    
    # Force/Torque settings
    ft_topic: "/ft_sensor/wrench"
    force_threshold: 5.0   # N
    torque_threshold: 0.5  # Nm
    
    # Adaptation parameters
    impedance:
      stiffness: [1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]
      damping: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
```

## Integration with LBR-Stack

The Adaptive Controller integrates with the LBR-Stack through the following steps:

1. **Controller Registration**:
   - Add the controller to the controller_manager configuration
   - Configure it as part of the controller chain
   
2. **Controller Startup**:
   - Load the adaptive controller alongside the joint_trajectory_controller
   - Configure the controller chain to pass commands through the adaptive controller
   
3. **Namespace Handling**:
   - Ensure all interactions respect the `/lbr/controller_manager` namespace
   - Use the correct namespace for all controller operations

Example controller configuration:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    adaptive_controller:
      type: adaptive_controller/AdaptiveController

controller_chainable_configuration:
  ros__parameters:
    controller_chain:
      - joint_trajectory_controller
      - adaptive_controller
```

## Development Workflow

1. Implement the FT-TwinCAT Bridge for force/torque data
2. Develop and test the Adaptive Controller with simulated data
3. Implement and test different adaptive strategies
4. Integrate with the LBR-Stack through proper controller configuration
5. Test with hardware-in-the-loop using the Beckhoff C6030

## Version Requirements

To ensure compatibility across the entire system, the following version requirements apply:

| Component              | Version                     | Notes                                          |
|------------------------|-----------------------------|-------------------------------------------------|
| Operating System       | Ubuntu 22.04.5 LTS (Jammy)  | Required for ROS2 Humble compatibility         |
| ROS2                   | Humble Hawksbill (0.10.0)   | LBR-Stack is developed and tested with Humble  |
| Python                 | 3.10.12 or newer            | Required for ROS2 Python interfaces            |
| C++ Compiler          | GCC 11.4.0 or newer         | C++17 support required for ros2_control        |
| TwinCAT                | 3.1.4024 or newer           | For EtherCAT master implementation             |
| pyads                  | 3.3.9 or newer              | For ADS communication with TwinCAT             |
| ros2_control           | Humble compatible           | For controller interface implementation        |
| LBR-Stack              | Compatible with Humble      | From https://github.com/lbr-stack/lbr_fri_ros2_stack |

## Conclusion

This architecture provides a clean, non-intrusive way to integrate adaptive control capabilities with the standard LBR-Stack. By maintaining clear boundaries between components and utilizing only public interfaces, we ensure that:

1. The LBR-Stack remains unmodified and maintainable
2. The adaptive control functionality is modular and extensible
3. The system can be easily configured for different scenarios
4. Future updates to the LBR-Stack will not break our adaptive functionality
