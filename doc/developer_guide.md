# Developer Guide: Adaptive Surface Contact Simulation

This guide provides information for developers who want to implement the Adaptive Surface Contact Simulation framework. The simulation is designed to test and visualize adaptive control algorithms for robot-environment interactions using force/torque feedback.

## Project Overview

The Adaptive Surface Contact Simulation is designed to:

1. Simulate a KUKA LBR IIWA14 robot interacting with virtual obstacles
2. Generate realistic force/torque feedback based on obstacle proximity and contact
3. Visualize robot motion, forces, and interaction states in RViz
4. Provide a framework to test adaptive control algorithms with simulated contacts

## System Requirements

- ROS2 Humble
- C++17 compatible compiler
- Gazebo 11 (optional for physics-based simulation)
- KUKA LBR Stack (for robot models and controllers)

## Architecture Overview

The project consists of three main components:

1. **Surface Contact Simulator**: Simulates force/torque feedback based on robot position and obstacle proximity
2. **Obstacle Manager**: Handles virtual obstacles and their visualization
3. **Simulation Visualization**: Provides enhanced visualization of the simulation state

These components communicate via ROS2 topics and services, and can be configured through parameter files.

### Component Interaction Diagram

```
┌────────────────────┐       ┌───────────────────┐       ┌──────────────────┐
│                    │       │                   │       │                  │
│  Surface Contact   │──────▶│  ROS2 Topics &    │◀─────▶│ Obstacle Manager │
│    Simulator       │◀──────│    Services       │       │                  │
│                    │       │                   │       └──────────────────┘
└────────────────────┘       │                   │
        ▲                    │                   │       ┌──────────────────┐
        │                    │                   │       │                  │
        │                    │                   │◀─────▶│   Simulation     │
        │                    │                   │       │  Visualization   │
        │                    └───────────────────┘       │                  │
        │                                                └──────────────────┘
        │                                                        ▲
        │                                                        │
┌───────┴───────────┐                                    ┌───────┴────────┐
│                   │                                    │                │
│    lbr-stack      │                                    │     RViz       │
│                   │                                    │                │
└───────────────────┘                                    └────────────────┘
```

## Integration with LBR Stack

The simulation framework relies on the KUKA LBR Stack for robot models, controller interfaces, and basic simulation capabilities. Integration points include:

### 1. Robot Description and Models

Use the robot description from the LBR Stack:

```python
# In launch files
robot_description = get_package_share_directory('lbr_description')
```

### 2. Controller Integration

The simulation should work with the joint_trajectory_controller provided by the LBR Stack:

```
Controller Topic: /lbr/joint_trajectory_controller/joint_trajectory
Controller Manager Namespace: /lbr/controller_manager
```

### 3. Joint State Information

Subscribe to joint states published by the robot (real or simulated):

```
Joint States Topic: /lbr/joint_states
```

### 4. Force/Torque Simulation

The Surface Contact Simulator should publish simulated force/torque data on the same topic that the real robot would use:

```
Force/Torque Topic: /lbr/force_torque_sensor
```

## Implementation Guidelines

When implementing the components, follow these guidelines:

### 1. Surface Contact Simulator

- Create a node that simulates force/torque data based on robot position and obstacle proximity
- Use the TF system to track the robot's end-effector position
- Implement different contact models (soft contact, hard contact, etc.)
- Publish simulated force/torque data that mimics the real sensor's behavior

Sample implementation approach:

```cpp
// Initialize publishers and subscribers
force_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/lbr/force_torque_sensor", 10);
joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/lbr/joint_states", 10,
    std::bind(&SurfaceContactSimulator::jointStateCallback, this, std::placeholders::_1));
```

### 2. Obstacle Manager

- Create a node that manages virtual obstacles in the environment
- Allow loading obstacle configurations from YAML files
- Visualize obstacles in RViz using marker arrays
- Provide services to add, remove, or modify obstacles at runtime

### 3. Simulation Visualization

- Create a node that enhances visualization of simulation data
- Subscribe to force/torque data and other relevant topics
- Create informative visual elements (force vectors, status displays, etc.)
- Implement time-series plotting for force data

## Configuration System

The simulation framework should use ROS2 parameters for configuration:

```yaml
# Sample parameter configuration
surface_contact_simulator:
  ros__parameters:
    force_threshold: 5.0
    approach_distance: 0.1
    contact_force_max: 10.0
    publish_frequency: 100.0
```

## Testing

Implement a comprehensive testing strategy:

1. **Unit Tests**: Test individual components and algorithms
2. **Integration Tests**: Verify the interaction between components
3. **System Tests**: Test the complete simulation with different scenarios

## Documentation

Document your implementation thoroughly:

1. **Code Comments**: Use Doxygen-style comments for classes, methods, and parameters
2. **README Files**: Provide clear instructions for building and running the simulation
3. **User Guides**: Create guides for configuring and using the simulation

## Best Practices

Follow these best practices for implementation:

1. **Code Style**: Follow the ROS2 C++ Style Guide
2. **Error Handling**: Implement robust error handling with informative messages
3. **Performance**: Optimize critical code paths for real-time performance
4. **Modularity**: Design components to be reusable and extensible
5. **Configuration**: Make all important parameters configurable
6. **Logging**: Use ROS2 logging for informative messages at appropriate levels
