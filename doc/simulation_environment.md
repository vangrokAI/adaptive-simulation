# Simulation Environment

## Overview

The simulation environment for the Adaptive Surface Contact project provides a comprehensive testing platform for development and validation without requiring physical hardware. This document describes the simulation architecture, setup procedures, and best practices.

## Gazebo Simulation

### Core Components

The simulation environment is built on Gazebo, a powerful physics-based 3D simulator:

1. **Robot Model**:
   - KUKA LBR IIWA14 URDF model with accurate kinematics
   - Ceiling-mounted configuration support
   - Joint limits and dynamics matching the real robot

2. **Physics Engine**:
   - ODE (Open Dynamics Engine) for accurate physical interactions
   - Configurable physics parameters (timestep, solver iterations)
   - Contact and friction modeling for surface interactions

3. **Environment Components**:
   - Contact surfaces with configurable properties
   - Task-specific fixtures and objects
   - Visual markers for trajectory and contact visualization

4. **Sensor Simulation**:
   - F/T sensor simulation with configurable noise models
   - Joint position, velocity, and effort feedback
   - Camera and vision sensor support (optional)

### Architecture

The simulation architecture integrates seamlessly with the ROS2 control system:

```
                     ┌─────────────────┐
                     │  ROS2 Control   │
                     │     System      │
                     └────────┬────────┘
                              │
                              ▼
┌─────────────┐      ┌─────────────────┐      ┌─────────────┐
│  Simulated  │      │    Gazebo       │      │  Simulated  │
│  F/T Sensor │◄────►│  Physics Engine │◄────►│   Robot     │
└─────────────┘      └─────────────────┘      └─────────────┘
                              │
                              ▼
                     ┌─────────────────┐
                     │  Visualization  │
                     │   (RViz/Gazebo) │
                     └─────────────────┘
```

## Setup and Configuration

### Prerequisites

- ROS2 Humble Hawksbill
- Gazebo Ignition 6.16.0+
- NVIDIA GPU recommended for complex simulations
- 8GB+ RAM for smooth operation

### Installation

1. **Gazebo Setup**:
   ```bash
   sudo apt install ros-humble-ros-gz
   sudo apt install ros-humble-ros-gz-sim-demos
   sudo apt install ros-humble-gz-sim6
   ```

2. **Robot Models**:
   ```bash
   # LBR models are included in the lbr-stack repository
   cd ~/ros2_ws
   git clone https://github.com/kroshu/lbr-stack.git src/lbr-stack
   cd ~/ros2_ws && colcon build --symlink-install
   ```

3. **Simulation Configuration**:
   ```bash
   # Copy simulation world files
   cp -r ~/ros2_ws/src/adaptive-simulation/simulation/worlds ~/.gz/
   
   # Update Gazebo model path
   echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/adaptive-simulation/simulation/models' >> ~/.bashrc
   source ~/.bashrc
   ```

### Launching the Simulation

Basic simulation launch with default parameters:

```bash
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa14 ceiling_mounted:=true use_sim_time:=true
```

Complete adaptive control simulation:

```bash
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_sim_time:=true ceiling_mounted:=true
```

## Simulation Features

### Time Management

The simulation uses simulated time, which is critical for proper ROS2 integration:

1. **Use Simulated Time**:
   - Always pass `use_sim_time:=true` to launch files
   - Ensures all ROS2 nodes synchronize with Gazebo time
   - Prevents timing issues with controllers and sensor data

2. **Simulation Speed**:
   - Real-time factor (RTF) can be adjusted for faster or slower simulation
   - Gazebo GUI: Physics → Time step (decrease for more accuracy, increase for speed)
   - Command line: `gz physics -t 0.001` (sets timestep to 1ms)

3. **Deterministic Simulation**:
   - Set `seed` parameter for reproducible physics behavior
   - Important for validation testing and debugging

### Robot Control

The simulated robot is controlled through the same interfaces as the real hardware:

1. **Controller Configuration**:
   - Uses identical controller configuration as real robot
   - `joint_trajectory_controller` for position control
   - `joint_impedance_controller` for compliant motion

2. **Controller Switching**:
   ```bash
   ros2 control set_controller_state -c /lbr/controller_manager joint_trajectory_controller inactive
   ros2 control set_controller_state -c /lbr/controller_manager joint_impedance_controller active
   ```

3. **Joint State Publishing**:
   - Simulated joint states available on `/lbr/joint_states`
   - Includes position, velocity, and effort data

### F/T Sensor Simulation

The simulation includes a detailed model of the ATI Axia80-M20 F/T sensor:

1. **Simulation Models**:
   - Physics-based: Uses Gazebo contact physics for realistic F/T responses
   - Pattern-based: Provides predefined F/T patterns (sine, constant, random)
   - Hybrid: Combines physics contact forces with pattern-based baseline

2. **Noise Models**:
   - Gaussian noise with configurable standard deviation
   - Low-frequency drift simulation
   - Outlier injection for robustness testing

3. **Configuration**:
   ```yaml
   ft_sensor_sim:
     ros__parameters:
       sim_mode: "physics"  # "physics", "pattern", or "hybrid"
       noise:
         enabled: true
         std_dev: [0.25, 0.25, 0.25, 0.025, 0.025, 0.025]  # N and Nm
       physics:
         contact_topic: "/gazebo/contact_points"
         link_name: "iiwa_link_ee"
   ```

## Working with the Simulation

### Visualization

Multiple visualization options are available:

1. **Gazebo GUI**:
   - Full 3D visualization of the simulation
   - Interactive camera controls
   - Physics and model inspection tools

2. **RViz**:
   - Customizable visualization of robot state, trajectories, and sensor data
   - Integration with ROS2 tools and plugins
   - Launch with: `ros2 launch lbr_bringup rviz.launch.py model:=iiwa14 use_sim_time:=true`

3. **Remote Visualization**:
   - For headless servers: VNC or X11 forwarding
   - Recommendation: VNC for better performance
   - Setup: `ros2 launch adaptive_simulation vnc_server.launch.py`

### Common Tasks

1. **Adding Contact Surfaces**:
   - Create or modify world files in `~/.gz/worlds/`
   - Add collision properties for accurate contact simulation
   - Adjust surface friction and compliance parameters

2. **Recording Simulation Data**:
   - Use ROS2 bag recording for sensor and control data
   - `ros2 bag record /ft_sensor/wrench /lbr/joint_states /tf`
   - Replay with `ros2 bag play` for analysis or regression testing

3. **Simulation Performance Tuning**:
   - Reduce visual complexity for better performance
   - Adjust physics timestep for balance of accuracy vs. speed
   - Use simplified collision meshes where possible

### Remote Access

For running simulations on remote servers:

1. **Headless Mode**:
   - Launch Gazebo without GUI: `gz sim -s` or set `gui:=false` in launch file
   - Useful for automated testing and CI/CD pipelines

2. **VNC Server**:
   - Provides remote GUI access
   - Setup included in the `vnc_server.launch.py` file

3. **SSH Port Forwarding**:
   - Forward ROS2 DDS discovery ports for remote connection
   - `ssh -L 8765:localhost:8765 user@server`

## Best Practices

1. **Simulation Fidelity**:
   - Start with simple scenarios and gradually increase complexity
   - Calibrate simulation parameters against real-world data when available
   - Document differences between simulation and real-world behavior

2. **Reproducibility**:
   - Use fixed random seeds for deterministic behavior
   - Version control simulation configurations
   - Document all parameters in README or parameter files

3. **Performance Optimization**:
   - Use simplified visual models with detailed collision models
   - Balance physics timestep with required accuracy
   - Consider reducing update rates for non-critical components

4. **Testing Methodology**:
   - Define clear test scenarios with success criteria
   - Create automated test scripts for regression testing
   - Validate simulation results against analytical models when possible

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Simulation crashes | Memory limits exceeded | Reduce model complexity, increase system memory |
| Unstable physics | Timestep too large | Reduce physics timestep, increase solver iterations |
| Controller instability | Simulation timing issues | Ensure `use_sim_time:=true` for all nodes |
| Robot falls through surfaces | Collision mesh issues | Check collision scales and margins |
| Delayed sensor readings | Incorrect topic configuration | Verify correct topic remappings in launch files |
| Poor visual performance | Graphics limitations | Run in headless mode, use simplified visual models |
