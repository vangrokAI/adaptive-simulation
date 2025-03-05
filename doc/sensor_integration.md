# Force/Torque Sensor Integration

## ATI Axia80-M20 F/T Sensor

The ATI Axia80-M20 is a high-precision 6-axis force/torque sensor that measures forces and torques in all three Cartesian coordinates. This document details its integration into the Adaptive Surface Contact Simulation project.

## Sensor Specifications

| Specification | Value |
|---------------|-------|
| Force Measurement Range | ±500 N (Fx, Fy), ±900 N (Fz) |
| Torque Measurement Range | ±20 Nm (Tx, Ty, Tz) |
| Resolution | 0.125 N (Force), 0.0075 Nm (Torque) |
| Single-Axis Overload | 900% |
| Interface | EtherCAT |
| Output Rate | Up to 1 kHz |
| Weight | 405g |
| Dimensions | 80mm diameter × 22mm height |

## Hardware Integration

### Physical Setup

The ATI Axia80-M20 F/T sensor is mounted between the robot's flange and the end-effector. The mounting follows these guidelines:

1. The sensor's reference coordinate system is aligned with the robot's TCP
2. Custom mounting plates ensure proper mechanical coupling
3. Cable routing minimizes mechanical stress on the sensor
4. Strain relief is provided for the EtherCAT cable

### Communication Architecture

```
[ATI Axia80-M20] --EtherCAT--> [Beckhoff C6030] --ADS--> [ROS2 System]
```

The communication setup involves:

1. The Beckhoff C6030 Industrial PC acts as an EtherCAT master
2. TwinCAT 3 software manages the EtherCAT communication
3. ADS (Automation Device Specification) protocol transfers data to ROS2
4. A ROS2 node translates ADS data to standard ROS2 topics

## Software Integration

### ROS2 Interface

The F/T sensor data is published as a standard `geometry_msgs/WrenchStamped` message:

```
# Message format: geometry_msgs/WrenchStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Wrench wrench
  geometry_msgs/Vector3 force
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 torque
    float64 x
    float64 y
    float64 z
```

### ROS2 Topics and Services

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | Raw F/T sensor data |
| `/ft_sensor/filtered_wrench` | `geometry_msgs/WrenchStamped` | Filtered F/T data |
| `/ft_sensor/bias` | `std_srvs/Trigger` | Service to tare the sensor |
| `/ft_sensor/status` | `diagnostic_msgs/DiagnosticStatus` | Sensor diagnostic information |

### Architecture Components

The software integration consists of the following components:

1. `ft_twincat_bridge` ROS2 package:
   - Communicates with TwinCAT via ADS
   - Converts raw sensor data to ROS2 messages
   - Provides filtering and bias compensation

2. `FTSensorInterface` class hierarchy:
   - Abstract base class for F/T sensor data providers
   - Concrete implementations for ADS and simulation
   - Plugin architecture for easy extension

3. `FTSensorSimulator` node:
   - Provides simulated F/T data for development and testing
   - Configurable simulation profiles (sine, constant, random)
   - Parameter-based control of simulation behavior

4. Data Processing:
   - Low-pass filtering for noise reduction
   - Bias compensation for zero reference
   - Coordinate frame transformation to match robot TCP

## Configuration

### TwinCAT 3 Configuration

TwinCAT 3 requires specific configuration for proper operation:

1. **EtherCAT Configuration**:
   - Installation of ATI ESI (EtherCAT Slave Information) file
   - Configuration of Process Data Objects (PDOs)
   - Setting up a cyclic task with ~1ms cycle time

2. **ADS Configuration**:
   - Setting up ADS routes to the ROS2 system
   - Configuring ADS NetID and Port
   - Adjusting firewall settings for ADS communication

### ROS2 Configuration

The ROS2 node is configured via parameters:

```yaml
ft_sensor_hardware:
  ros__parameters:
    data_source: "ads"  # or "sim" for simulation
    twincat:
      twincat_ip: "192.168.2.100"
      ads_net_id: "192.168.2.100.1.1"
      ads_port: 851
      update_rate: 1000  # Hz
    filtering:
      enabled: true
      cutoff_frequency: 10.0  # Hz
    frame_id: "ft_sensor_frame"
```

## Simulation Support

For development without physical hardware, the system supports F/T sensor simulation:

### Simulation Modes

1. **Sine Wave Simulation**:
   - Generates sinusoidal F/T values
   - Configurable frequency, amplitude, and phase
   - Good for testing oscillating contact forces

2. **Constant Simulation**:
   - Provides constant F/T values
   - Configurable offset and noise level
   - Suitable for static load testing

3. **Random Simulation**:
   - Generates random F/T values within bounds
   - Configurable bounds and noise characteristics
   - Useful for robustness testing

### Simulation Parameters

```yaml
ft_sensor_sim:
  ros__parameters:
    sim_mode: "sine"  # "sine", "constant", or "random"
    publish_rate: 100  # Hz
    sine:
      amplitude: [10.0, 10.0, 20.0, 1.0, 1.0, 1.0]  # [Fx, Fy, Fz, Tx, Ty, Tz]
      frequency: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # Hz
      phase: [0.0, 1.57, 3.14, 0.0, 1.57, 3.14]  # radians
    constant:
      value: [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]  # [Fx, Fy, Fz, Tx, Ty, Tz]
      noise_amplitude: [0.5, 0.5, 0.5, 0.05, 0.05, 0.05]
    random:
      min_value: [-20.0, -20.0, -30.0, -2.0, -2.0, -2.0]
      max_value: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0]
```

## Calibration and Bias Compensation

For accurate F/T measurements, proper calibration is essential:

1. **Sensor Calibration**:
   - Factory calibration parameters are stored in the sensor
   - No additional calibration is typically needed

2. **Bias Compensation**:
   - Zero-point calibration to compensate for gravity and mounting forces
   - Triggered via the `/ft_sensor/bias` service
   - Should be performed when the sensor is under no external load

3. **Tool Weight Compensation**:
   - The weight of the end-effector can be automatically compensated
   - Requires the mass and center of gravity of the tool
   - Configurable via parameters

## Integration with Adaptive Control

The F/T sensor is a crucial component of the adaptive surface contact controller:

1. **Contact Detection**:
   - Force thresholds for detecting surface contact
   - Direction-aware contact determination
   - Filtering to avoid false positives

2. **Force Control**:
   - Feedback loop using F/T measurements
   - Compliance parameters for different surfaces
   - Safety limits for maximum allowable forces

3. **Adaptive Response**:
   - Adjusting robot behavior based on measured forces
   - Surface stiffness estimation
   - Gradual approach and retreat strategies

## Troubleshooting

Common issues and their solutions:

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| No F/T data | EtherCAT communication issue | Check cables, TwinCAT status |
| Incorrect readings | Missing bias compensation | Run the bias service |
| Noisy data | Environmental vibration | Adjust filter parameters |
| Sensor overload | Excessive force/torque | Check application limits |
| Slow update rate | Communication bottleneck | Verify network setup, ADS parameters |
