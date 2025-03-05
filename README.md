# Adaptive Surface Contact Simulation

## 🚀 Quick Start Guide for New Developers

Welcome to the Adaptive Surface Contact Simulation project! This guide will help you get started quickly and efficiently.

### 1. Project Overview

This project provides a simulation environment for the adaptive surface contact module of the ceiling-mounted KUKA LBR IIWA14 robot with ATI Axia80-M20 Force/Torque sensor integration via EtherCAT. It enables development of robust contact control algorithms for industrial applications.

### 2. Development Setup

```bash
# Clone this repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/vangrokAI/adaptive-simulation.git

# Install LBR-Stack (required dependency)
git clone https://github.com/kroshu/lbr-stack.git

# Install dependencies
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rviz2 ros-humble-joint-state-publisher-gui

# Build workspace
cd /opt/grok/adaptive-simulation
colcon build

# Source environment
source /opt/grok/adaptive-simulation/install/setup.bash
```

### 3. Run Basic Simulation

```bash
# Start simulation with default parameters
ros2 launch adaptive_simulation simulation.launch.py

# For hardware mode with real F/T sensor
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_hardware:=true
```

### 4. Documentation Map

To understand the system better, follow this documentation sequence:

1. [Use Case Description](doc/adaptive_surface_contact_usecase.md) - Overall project goals and requirements
2. [Controller Architecture](doc/controller_architecture.md) - Non-intrusive controller design
3. [KUKA ROS2 Server](doc/kuka_ros2_server.md) - Integration with the robot controller
4. [Developer Guide](doc/developer_guide.md) - Detailed technical implementation

## System Architecture

The system consists of four main components that work together to provide a robust force-sensitive control solution:

1. **Ceiling-mounted KUKA LBR IIWA14 Robot** - Controlled via FRI
2. **ATI Axia80-M20 F/T Sensor** - Connected via EtherCAT
3. **Beckhoff C6030 Industrial PC** - Running TwinCAT 3 for EtherCAT management
4. **ROS2 Control System** - For high-level control and simulation

## Key Features

- Simulation of the ceiling-mounted KUKA LBR IIWA14 robot with ATI Axia80-M20 F/T sensor
- Configurable obstacle environment for top-down contact testing
- Advanced visualization of sensor and motion data in RViz
- Seamless integration with the LBR-Stack through non-intrusive interfaces
- Configurable force-feedback control algorithms
- Support for both simulated and real hardware sensors

## System Requirements

| Component              | Version                     | Notes                                         |
|------------------------|-----------------------------|-------------------------------------------------|
| Operating System       | Ubuntu 22.04.5 LTS (Jammy)  | Required for ROS2 Humble compatibility        |
| ROS2                   | Humble Hawksbill (0.10.0)   | LBR-Stack is developed and tested with Humble |
| Python                 | 3.10.12 or newer            | Required for ROS2 Python nodes                |
| C++ Compiler          | GCC 11.4.0 or newer         | C++17 support required                        |
| Gazebo                 | Ignition Gazebo 6.16.0      | For simulation environment                    |
| TwinCAT 3              | 3.1.4024 or newer           | Required for EtherCAT communication           |

## Project Structure

```
adaptive-simulation/
├── config/              # Configuration files for simulation and hardware
├── doc/                 # Documentation files
│   ├── adaptive_surface_contact_usecase.md  # Use case description
│   ├── controller_architecture.md           # Controller design
│   ├── kuka_ros2_server.md                  # KUKA integration
│   └── developer_guide.md                   # Development guide
├── launch/              # ROS2 launch files
├── src/                 # Source code
│   ├── ft_twincat_bridge/   # Force/Torque sensor interface
│   └── adaptive_simulation/ # Simulation environment
└── urdf/                # Robot and sensor models
```

## Development Workflow

1. **Understanding the System**
   - Start by reading the documentation in the suggested order
   - Explore the code structure and ROS2 nodes

2. **Development Environment Setup**
   - Install dependencies including the LBR-Stack
   - Configure your development environment

3. **Testing in Simulation**
   - Start with simulated sensors using `use_sim:=true`
   - Test your implementation in the simulation environment

4. **Hardware Integration**
   - Configure the TwinCAT system (if available)
   - Test with real hardware using `use_hardware:=true`

## F/T Sensor Integration

The ATI Axia80-M20 F/T sensor (EtherCAT version) is integrated through the following pipeline:

```
ATI Axia80-M20 F/T Sensor → Beckhoff C6030 (TwinCAT 3) → ADS → ROS2 System
```

Features:
- Measurement range: ±500 N (Fx, Fy), ±900 N (Fz), ±20 Nm (Tx, Ty, Tz)
- Resolution: 0.25 N for forces, 0.01 Nm for torques
- Update rate: 500 Hz for real-time force control

## Testing and Operation

### Simulation Mode

```bash
# Gazebo simulation with ceiling-mounted robot
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa14 ctrl:=joint_trajectory_controller ceiling_mounted:=true use_sim_time:=true

# Complete simulation with F/T sensor
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_sim_time:=true
```

### Hardware Mode

```bash
# Start the hardware driver for LBR IIWA14 (remember the correct namespace)
ros2 launch lbr_bringup hardware.launch.py ctrl:=joint_trajectory_controller model:=iiwa14 ceiling_mounted:=true

# Verify controller status
ros2 control list_controllers -c /lbr/controller_manager

# Start with real F/T sensor
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_hardware:=true
```

## Common Issues and Solutions

1. **Controller not found** - Ensure you're using the correct namespace: `/lbr/controller_manager`
2. **F/T sensor data not available** - Check TwinCAT connection and ADS routes
3. **Simulation crashes** - Verify correct ROS2 and Gazebo versions

## Getting Help

For questions and support, please contact:

- Project Team: <email@example.com>
- Development Lead: <lead@example.com>

## ATI Axia80-M20 EtherCAT Integration with Beckhoff C6030

The integration of the ATI Axia80-M20 EtherCAT F/T sensor is accomplished through a Beckhoff C6030 Industrial PC with TwinCAT 3 as the EtherCAT master. This architecture offers several advantages:

1. **Real-time communication** through the deterministic EtherCAT protocol
2. **Industry standard** for reliable automation
3. **Clean system separation** between real-time control and ROS2
4. **Flexible configuration options** for various use cases

### Architecture of the EtherCAT-ROS2 Integration

```
ATI Axia80-M20 F/T Sensor (EtherCAT) → Beckhoff C6030 (TwinCAT 3) → ADS → ROS2 System
```

The data flow works as follows:
- The ATI sensor communicates via EtherCAT with the Beckhoff C6030
- TwinCAT 3 reads the sensor data in real-time
- The data is transferred via ADS (Automation Device Specification) to the ROS2 system
- A ROS2 node converts the ADS data into standard ROS2 topics (geometry_msgs/WrenchStamped)

### ADS Communication

ADS (Automation Device Specification) is a Beckhoff protocol that enables communication with TwinCAT systems via Ethernet:

- **ADS NetID**: Unique identification of the TwinCAT system (e.g., `192.168.2.100.1.1`)
- **ADS Port**: Identification of the service within TwinCAT (e.g., 851 for PLC Runtime)
- **Variable access**: Direct read and write operations on TwinCAT variables
- **Platform independence**: Works with Linux and ROS2 through the pyads library

### F/T Sensor Data Simulation

For development and testing, F/T sensor data can be simulated without requiring the physical sensor or TwinCAT system:

1. **Interface-based architecture**:
   - Common data interface for simulated and real data
   - Seamless switching between simulation and hardware
   - Support for various simulation profiles (sine, constant, random)

2. **Simulation parameters**:
   - `data_source`: Selects between `sim` (simulated) and `ads` (real)
   - `sim_mode`: Selects the simulation profile (`sine`, `constant`, `random`)
   - `publish_rate`: Configures the publishing rate in Hz

3. **Launch parameters**:
   ```bash
   # For simulated F/T data
   ros2 launch adaptive_simulation ft_sensor.launch.py use_sim:=true
   
   # For real F/T data via ADS
   ros2 launch adaptive_simulation ft_sensor.launch.py use_sim:=false twincat_netid:=192.168.2.100.1.1
   ```

4. **ROS2 implementation**:
   A complete interface-based implementation is available in the `src/ft_twincat_bridge` directory. This includes:
   - Abstract base class for data providers
   - Concrete implementations for ADS and simulation
   - Complete ROS2 configuration and parameters
   - Launch files for easy integration

### Setting Up the TwinCAT Environment

Setting up the C6030 with TwinCAT 3 requires the following steps:

1. **Installing the ATI ESI file**:
   - Obtain the EtherCAT Slave Information (ESI) from ATI support
   - Install in TwinCAT 3 in the directory `C:\TwinCAT\3.1\Config\Io\EtherCAT`

2. **Configure EtherCAT master**:
   - Add ATI Axia80-M20 as an EtherCAT slave
   - Map process data (PDO) for F/T values
   - Create a cyclic task with ~1ms cycle time

3. **Set up ADS route**:
   - Add route to the ROS2 system
   - Adjust firewall rules for ADS (Port 48898)
   - Configure ADS NetID and AMS Routes

### Starting with the Real LBR Robot

```bash
# Start the hardware driver for the LBR IIWA14 with ceiling_mounted parameter
ros2 launch lbr_bringup hardware.launch.py ctrl:=joint_trajectory_controller model:=iiwa14 ceiling_mounted:=true

# Check controller status (note the correct namespace)
ros2 control list_controllers -c /lbr/controller_manager
```

### Starting in Simulation

```bash
# Gazebo simulation with ceiling-mounted robot
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa14 ctrl:=joint_trajectory_controller ceiling_mounted:=true use_sim_time:=true
```

### Starting with F/T Sensor

```bash
# For complete simulation with ATI Axia80-M20 F/T sensor
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_sim_time:=true ceiling_mounted:=true

# For operation with a real F/T sensor (EtherCAT version)
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_hardware:=true ceiling_mounted:=true use_sim_time:=false

# Configuration of the TwinCAT 3 EtherCAT connection
# The IP address is only used for diagnostic purposes, not for the actual EtherCAT communication
ros2 param set /ft_sensor_hardware twincat.twincat_ip 192.168.2.100
ros2 param set /ft_sensor_hardware twincat.ads_net_id "192.168.2.100.1.1"
```
