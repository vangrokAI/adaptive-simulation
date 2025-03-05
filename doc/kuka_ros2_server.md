# KUKA ROS2 Server

## Overview

The KUKA ROS2 Server is a critical component in the adaptive surface contact system that runs directly on the KUKA Sunrise controller. It establishes and maintains the Fast Robot Interface (FRI) connection between the KUKA robot and the ROS2 environment, enabling real-time communication and control.

This document describes the implementation, configuration, and integration of the ROS2Server Java application that runs on the KUKA Sunrise controller.

## Architecture

The ROS2Server operates as a bridge between the KUKA robot controller and the ROS2-based control system:

```
┌────────────────────┐      FRI       ┌────────────────────┐
│                    │ ◄───Protocol───► │                    │
│   KUKA Controller  │  (UDP/1kHz)    │   ROS2 System      │
│  (Sunrise OS)      │                │  (Ubuntu 22.04)    │
│                    │                │                    │
│  [ROS2Server.java] │                │  [lbr_fri_ros2_stack] │
│                    │                │                    │
└────────────────────┘                └────────────────────┘
```

### Key Components

1. **ROS2Server Java Application**: Runs on the KUKA Sunrise controller
2. **FRI Communication Layer**: Enables real-time data exchange with minimal latency
3. **Position Control Mode**: Provides stable control interface for the ROS2 system
4. **Error Recovery Mechanisms**: Automatically handles communication interruptions and robot errors

## Implementation Details

### Configuration Parameters

| Parameter            | Default Value    | Description                                      |
|----------------------|------------------|--------------------------------------------------|
| ROS2_CLIENT_IP       | 192.170.10.5     | IP address of the ROS2 system (FRI client)       |
| SEND_PERIOD_MS       | 1                | FRI cycle time in milliseconds (1000Hz)          |
| MAX_CONNECTION_ATTEMPTS | 100           | Maximum number of connection retry attempts      |
| RETRY_DELAY_SEC      | 5                | Delay between connection attempts (seconds)      |

### Control Mode

The ROS2Server uses the KUKA `PositionControlMode` by default, which provides:

- Highest stability for ROS2 integration
- Direct control of joint positions
- Compatible with the standard `joint_trajectory_controller` in ROS2

### Error Handling

The application includes robust error handling mechanisms:

1. **Robot State Errors**: Handles safety stops and protective stops
2. **FRI Communication Errors**: Automatically attempts to reestablish connection
3. **Runtime Errors**: Logs and manages unexpected exceptions
4. **Critical Errors**: Terminates with appropriate error message

## Integration with LBR-Stack

The KUKA ROS2Server is designed to work seamlessly with the [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack):

1. The server initializes and establishes an FRI connection
2. The LBR-Stack connects as an FRI client to receive robot state and send commands
3. Communication occurs at up to 1kHz (1ms cycle time) for real-time control

## Installation and Setup

### Prerequisites

- KUKA LBR IIWA14 robot with KUKA Sunrise OS
- Access to KUKA Sunrise Workbench for deploying applications
- Network configuration for FRI communication

### Deployment Steps

1. **Prepare the Java Application**:
   - Create a new Sunrise project or use an existing one
   - Add the ROS2Server.java to the `src/lbr_fri_ros2` package
   - Configure the IP address and other parameters as needed

2. **Install on the KUKA Controller**:
   - Deploy the application to the KUKA controller
   - Set the application to auto-start with the robot

3. **Network Configuration**:
   - Configure the network settings on the KUKA controller
   - Ensure the ROS2 system is on the same network and using the specified IP

## Security Considerations

- FRI uses a local network connection and does not implement authentication
- The connection should be on a dedicated, isolated network
- Access controls should be implemented at the network level

## Troubleshooting

### Common Issues

1. **Connection Failures**:
   - Verify network settings on both KUKA and ROS2 system
   - Check firewall settings
   - Ensure correct IP addresses are configured

2. **Robot Error States**:
   - Reset robot after protective stop
   - Check safety configuration
   - Verify that motion commands are within limits

3. **Performance Issues**:
   - Adjust the SEND_PERIOD_MS parameter based on network capabilities
   - Monitor jitter and latency values from the FRI diagnostics
   - Ensure the ROS2 system can handle the communication rate

## Conclusion

The KUKA ROS2Server provides a robust and reliable interface between the KUKA robot and the ROS2-based adaptive surface contact system. It ensures stable communication through the FRI protocol while implementing fault tolerance and automatic recovery mechanisms.

When combined with the ft_twincat_bridge for force/torque sensing and the adaptive controller, this creates a complete system for sensitive force-controlled manipulation using the ceiling-mounted KUKA LBR IIWA14 robot.
