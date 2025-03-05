# Adaptive Surface Contact - Use Case Documentation

## Overview

This document describes the use case for the adaptive surface contact module developed for the ceiling-mounted KUKA LBR IIWA14 (R820) robot. The objective is to develop a robust, fault-resistant, and industry-ready system that enables the robot to sensitively respond to environmental contacts and adjust its movement accordingly. The system is designed as a standalone solution with industrial-grade components.

## System Requirements

To ensure compatibility with the LBR-Stack and the industrial components, the following system requirements must be met:

### Software Requirements

| Component              | Version                     | Notes                                         |
|------------------------|-----------------------------|-------------------------------------------------|
| Operating System       | Ubuntu 22.04.5 LTS (Jammy)  | Required for ROS2 Humble compatibility        |
| ROS2                   | Humble Hawksbill (0.10.0)   | LBR-Stack is developed and tested with Humble |
| Python                 | 3.10.12 or newer            | Required for ROS2 Python nodes                |
| C++ Compiler          | GCC 11.4.0 or newer         | C++17 support required                        |
| Gazebo                 | Ignition Gazebo 6.16.0      | For simulation environment                    |
| TwinCAT 3              | 3.1.4024 or newer           | Required for EtherCAT communication           |
| pyads                  | 3.3.9 or newer              | For ADS communication with TwinCAT            |

### Hardware Requirements

| Component              | Model/Specification            | Notes                                         |
|------------------------|--------------------------------|-------------------------------------------------|
| Robot                  | KUKA LBR IIWA14 (R820)         | With ceiling-mounted configuration            |
| F/T Sensor             | ATI Axia80-M20                 | With EtherCAT interface                       |
| Industrial PC          | Beckhoff C6030                 | With TwinCAT 3 runtime                        |
| Network                | Gigabit Ethernet               | For EtherCAT and FRI communication            |

### Development Environment

The development environment should include:

1. ROS2 Humble workspace setup
2. Access to the LBR-Stack repositories
3. TwinCAT 3 development environment (for EtherCAT/ADS development)
4. Python development tools
5. C++ development tools for ROS2 packages

## System Components

### Hardware

1. **KUKA LBR IIWA14 (R820)** 
   - 7-axis robot arm in ceiling-mounted configuration
   - Built-in torque sensors in each joint
   - Maximum payload: 14 kg
   - FRI (Fast Robot Interface) for external control

2. **ATI Axia80-M20 F/T Sensor (EtherCAT Version)**
   - High-sensitivity force/torque sensor at the robot end-effector
   - 6-axis measurement (Fx, Fy, Fz, Tx, Ty, Tz)
   - High accuracy and low noise characteristics
   - Measurement range: ±500 N (Fx, Fy), ±900 N (Fz), ±20 Nm (Tx, Ty, Tz)
   - Industrial EtherCAT communication interface

3. **Beckhoff C6030 Industrial PC**
   - Compact industrial PC for EtherCAT master functionality
   - TwinCAT 3 real-time control system
   - ADS communication interface to ROS2
   - High-performance Intel Core i5/i7 CPU

### Software

1. **KUKA ROS2 Server**
   - Java application running on the KUKA Sunrise controller
   - Establishes FRI connection with the ROS2 system
   - Handles communication errors and recovery
   - [Detailed documentation](kuka_ros2_server.md)

2. **ROS2 Control Framework**
   - Utilization of Joint Trajectory Controller
   - Integrated velocity scaling
   - Real-time communication with the robot

3. **LBR-Stack**
   - Driver interface to the KUKA robot
   - Hardware interface for FRI
   - Gazebo simulation support

4. **Adaptive Simulation Package**
   - Simulation environment for the Surface Contact Test
   - Configurable robot paths and obstacles
   - Comprehensive visualization in RViz
   - Integration with Beckhoff TwinCAT through ADS

## Functional Use Case: Surface Contact Test

### Objective
Development of an adaptive robot control system that continuously monitors environmental contacts and adjusts movement accordingly to safely navigate to the target. The ceiling-mounted configuration presents unique challenges and opportunities for surface contact from above.

### Behavior

1. **Continuous Sensor Monitoring**
   - The F/T sensor at the end-effector constantly monitors forces and torques
   - Real-time filtering and processing of sensor data via EtherCAT and TwinCAT
   - Detection of even minor contacts with high precision

2. **Velocity Adaptation**
   - Upon detection of contact forces, movement speed is reduced
   - Speed reduction occurs proportionally to the measured force
   - Minimum speed is configurable (default: 10% of normal speed)

3. **Goal-Directed Movement**
   - The robot continuously attempts to reach its predefined waypoint
   - Upon contact, velocity is reduced while direction initially remains unchanged
   - Movement is only stopped when the force threshold is exceeded

4. **Force Threshold and Retraction**
   - When a defined force threshold is exceeded (default: 10N):
     * Movement is completely stopped
     * The robot retracts in the direction opposite to the contact force
     * Retraction continues until force measurement falls below a threshold
   - After retraction, the current waypoint is marked as successfully completed

5. **Continuation of Movement**
   - After successful retraction or reaching the waypoint, movement to the next waypoint continues
   - The entire process is repeated for all waypoints in the programmed path

## Simulation Requirements

### Simulated Environment

1. **Robot Model**
   - Complete kinematic and dynamic model of the ceiling-mounted KUKA LBR IIWA14
   - Accurate mass parameters and joint properties
   - Integration of the ATI Axia80-M20 sensor at the end-effector
   - Proper orientation for ceiling-mounted configuration

2. **Obstacle Configuration**
   - Configurable obstacle geometries (boxes, cylinders, planes, etc.)
   - Positioning and orientation of obstacles in the workspace
   - Adaptable material properties (rigid, elastic)
   - Specialized configurations for top-down contact testing

3. **Force Simulation**
   - Realistic simulation of contact forces between robot and obstacles
   - Consideration of friction and material properties
   - Noise simulation for realistic sensor behavior
   - Simulated EtherCAT communication characteristics

### Motion Planning and Control

1. **Waypoint Definition**
   - Configurable list of waypoints in joint space or Cartesian space
   - Temporal parameters for movements (velocity, acceleration)
   - Special considerations for ceiling-mounted configuration
   - Optionale Zielvorgaben für Orientierung und Position

2. **Adaptive Regelung**
   - Implementierung des kraftbasierten Geschwindigkeitsskalierungsalgorithmus
   - Parametrierbare Regelungsparameter (Kraftschwellwerte, Skalierungsfaktoren)
   - Realistische Reaktionszeiten des Reglers

### Visualisierung

1. **RViz Integration**
   - Complete visualization of the robot and environment
   - Display of contact forces as vectors at the end-effector
   - Color-coded display of force intensities
   - Specialized visualization for ceiling-mounted configuration

2. **Data Visualization**
   - Real-time charts for force and velocity profiles
   - Status displays for phases (approach, contact, retraction)
   - Visualization of planned and actual trajectory
   - EtherCAT and ADS communication status monitoring

3. **Interactive Elements**
   - Ability to interactively adjust obstacles
   - Real-time parameter adjustment during simulation
   - EtherCAT/ADS communication diagnostics interface
   - Signal quality and latency visualization
   - Configuration menus for test parameters
   - Controls for start, pause, and reset of the simulation

## Data Collection and Analysis

1. **Data Storage**
   - Recording of all relevant data (joint positions, velocities, forces)
   - Timestamps for precise temporal analysis
   - Export to common formats (CSV, ROS bags)
   - Logging of EtherCAT/ADS communication metrics

2. **Analysis Tools**
   - Tools for evaluating simulation results
   - Comparison of different parameter configurations
   - Statistical evaluations (max. forces, average velocities)
   - Signal quality assessment for EtherCAT communication

3. **Metric Capture**
   - Recording of performance metrics (time to target, max. forces experienced)
   - Evaluation of path accuracy
   - Communication latency measurements
   - Energy efficiency analysis
   - EtherCAT communication reliability metrics

## Implementation Steps

1. **Setup of the Simulation Environment**
   - Integration of the existing LBR-Stack into the simulation
   - Creation of simulated obstacles with top-down approach considerations
   - Configuration of the F/T sensor simulation with EtherCAT characteristics
   - Implementation of ADS communication simulation layer

2. **Implementation of the Adaptive Control**
   - Development of a standalone control system for surface contact
   - Integration with TwinCAT 3 ADS communication layer
   - Optimization for the ceiling-mounted configuration
   - Validation of control parameters through simulation

3. **Visualization Components**
   - Development of extended RViz plugins
   - Integration of charts and status displays
   - Real-time force visualization
   - EtherCAT/ADS communication status visualization

4. **User Interface and Configuration**
   - GUI for configuring simulation parameters
   - Controls for test execution
   - User-friendly presentation of results
   - TwinCAT configuration interface

## Development Setup

### LBR-Stack Integration

The adaptive surface contact simulation depends on the lbr_fri_ros2_stack, which can be integrated into the development environment independently. Follow these steps to set up the required dependencies:

1. **Obtain the LBR-Stack**
   - Clone the official repository: `git clone https://github.com/lbr-stack/lbr_fri_ros2_stack`
   - Follow the build and installation instructions in the repository README
   - Ensure compatibility with ROS2 Humble Hawksbill

2. **Configure FRI Settings**
   - Match FRI Client version with your robot's FRI version
   - Configure network settings according to your KUKA robot configuration
   - Verify connection using the diagnostic tools provided in the lbr-stack

3. **Controller Configuration**
   - Remember to use the correct controller manager namespace for all ros2_control commands: `/lbr/controller_manager`
   - Example: `ros2 control list_controllers -c /lbr/controller_manager`
   - The joint_trajectory_controller will be the primary controller for the adaptive surface contact simulation

### EtherCAT and TwinCAT Integration

1. **Beckhoff C6030 Setup**
   - Install TwinCAT 3 on the Industrial PC
   - Configure TwinCAT project for EtherCAT communication with the ATI Axia80-M20
   - Set up ADS communication parameters for ROS2 integration

2. **Testing the Integration**
   - Verify EtherCAT communication with the F/T sensor
   - Test the ADS connection between the TwinCAT system and ROS2 environment
   - Validate data flow from sensor to ROS2 topics

### Adaptive Controller Architecture

1. **Non-Intrusive Controller Design**
   - Develop a standalone adaptive controller package outside the standard LBR-Stack
   - Interact with the LBR-Stack only through its public ROS2 interfaces
   - Maintain clear boundaries between core robot functionality and adaptive behavior

2. **Controller Integration Approach**
   - Use a controller_manager::ChainableControllerInterface to create a cascaded controller
   - Subscribe to the F/T sensor data from the ft_twincat_bridge
   - Modify the joint_trajectory_controller's commands based on the F/T feedback
   - Utilize Controller Manager interfaces rather than modifying the core stack

3. **Adaptive Behavior Layer**
   - Implement adaptive algorithms in a separate package: `adaptive_controller`
   - Design plugin architecture for different adaptive strategies
   - Connect to the LBR-Stack through standardized ROS2 controller interfaces
   - Create a clean API for configuration and parameter tuning

## Integration Requirements

The simulation environment must be compatible with the existing LBR-Stack and integrate seamlessly with the Beckhoff C6030. This includes:

1. **ROS2 Interface**
   - Compatibility with the ROS2 interfaces of the LBR-Stack
   - Use of standardized message types
   - Correct namespace structure (/lbr/controller_manager)
   - Support for ADS communication through pyads

2. **Controller Integration**
   - Support for the joint_trajectory_controller
   - Correct integration of velocity scaling
   - Compatibility with the existing controller manager
   - Interface for TwinCAT EtherCAT real-time control

3. **Gazebo/RViz Integration**
   - Shared use of models and configurations
   - Synchronization between Gazebo and RViz visualization
   - Consistent use of coordinate systems and transformations
   - Support for ceiling-mounted robot configuration
