# Launch System Documentation

## Overview

The launch system for the Adaptive Surface Contact Simulation project provides a flexible and configurable way to start the simulation environment, the real hardware system, or a hybrid setup. This document explains the available launch files, their parameters, and common usage scenarios.

## Key Launch Files

### Core Launch Files

| Launch File | Package | Description |
|-------------|---------|-------------|
| `hardware.launch.py` | `lbr_bringup` | Starts the real KUKA LBR IIWA14 robot with FRI communication |
| `gazebo.launch.py` | `lbr_bringup` | Starts the Gazebo simulation environment with the robot model |
| `surface_contact_test.launch.py` | `adaptive_control` | Launches the adaptive surface contact test |
| `ft_sensor.launch.py` | `adaptive_simulation` | Starts the F/T sensor node (real or simulated) |
| `surface_contact_with_ft.launch.py` | `adaptive_simulation` | Complete launch file for adaptive control with F/T sensing |

### Important Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim_time` | Boolean | `false` | Use simulated time when `true`, required for Gazebo |
| `ceiling_mounted` | Boolean | `false` | Configure for ceiling-mounted robot when `true` |
| `model` | String | `iiwa14` | Robot model to use, options: `iiwa7`, `iiwa14` |
| `ctrl` | String | `joint_trajectory_controller` | Controller to load |
| `use_sim` | Boolean | `true` | Use simulated F/T sensor when `true` |
| `use_hardware` | Boolean | `false` | Use real hardware when `true` |
| `sim_mode` | String | `sine` | F/T simulation mode: `sine`, `constant`, `random` |
| `publish_rate` | Integer | `100` | Rate (Hz) for publishing F/T data |
| `twincat_netid` | String | `192.168.2.100.1.1` | ADS NetID of the TwinCAT system |

## Launch Scenarios

### 1. Complete Simulation Setup

To start a complete simulation environment with Gazebo, the ceiling-mounted robot, and simulated F/T sensor:

```bash
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_sim_time:=true ceiling_mounted:=true
```

### 2. Hardware Setup with Real Robot

To start with the real KUKA LBR IIWA14 robot in ceiling-mounted configuration:

```bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=joint_trajectory_controller model:=iiwa14 ceiling_mounted:=true
```

### 3. Hardware Setup with Real F/T Sensor

To use the real ATI Axia80-M20 F/T sensor with TwinCAT integration:

```bash
ros2 launch adaptive_simulation ft_sensor.launch.py use_sim:=false twincat_netid:=192.168.2.100.1.1
```

### 4. Complete Hardware Setup

For the full hardware setup with real robot and real F/T sensor:

```bash
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py use_hardware:=true ceiling_mounted:=true use_sim_time:=false
```

## Namespace Considerations

When interacting with controllers, always specify the correct controller manager namespace:

```bash
# Correct: Specifying the controller manager namespace
ros2 control list_controllers -c /lbr/controller_manager

# Incorrect: Missing namespace
ros2 control list_controllers
```

## Launch File Hierarchy

The launch files are organized hierarchically, with higher-level launch files including lower-level ones:

```
surface_contact_with_ft.launch.py
├── gazebo.launch.py or hardware.launch.py
└── ft_sensor.launch.py
    └── Various nodes for F/T sensing
```

This structure allows for maximum flexibility and reuse of components.

## Debugging Launch Files

To debug launch file issues, use the `--show-args` flag to see all resolved parameters:

```bash
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py --show-args
```

For more detailed launch information, including node configuration:

```bash
ros2 launch adaptive_simulation surface_contact_with_ft.launch.py --show-args --debug
```

## Environment Variables

The following environment variables impact the launch system behavior:

| Variable | Purpose |
|----------|---------|
| `GAZEBO_MODEL_PATH` | Additional paths for Gazebo models |
| `TWINCAT_ADS_ADDRESS` | Override default TwinCAT IP address |
| `ROS_DOMAIN_ID` | ROS2 domain ID for network isolation |
| `RMW_IMPLEMENTATION` | ROS middleware implementation |

## Common Launch Errors and Solutions

| Error | Possible Cause | Solution |
|-------|----------------|----------|
| "Controller not found" | Incorrect namespace | Specify `-c /lbr/controller_manager` |
| "Unable to connect to FRI" | Network issue or robot offline | Check IP configuration and robot status |
| "TwinCAT connection failed" | ADS configuration issue | Verify firewall settings and ADS NetID |
| "Gazebo model not found" | Missing model paths | Ensure correct `GAZEBO_MODEL_PATH` |
| "F/T data not publishing" | Sensor configuration | Check topics with `ros2 topic list` |
