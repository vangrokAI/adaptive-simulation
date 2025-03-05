#!/bin/bash
# Script to start the Adaptive Surface Contact Simulation
# This script handles the startup process, including proper process management

# Terminal colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Print header
echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}   Adaptive Surface Contact Simulation       ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Function to check for running ROS2 processes
check_ros2_processes() {
    local running_procs=$(ps aux | grep -E "ros2|gazebo|rviz" | grep -v grep | wc -l)
    if [ $running_procs -gt 0 ]; then
        echo -e "${YELLOW}Found existing ROS2 processes:${NC}"
        ps aux | grep -E "ros2|gazebo|rviz" | grep -v grep
        echo -e "${YELLOW}Would you like to terminate these processes? [Y/n]${NC}"
        read -r response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY]|"")$ ]]; then
            echo -e "${YELLOW}Terminating ROS2 processes...${NC}"
            pkill -f "ros2"
            pkill -f "gazebo"
            pkill -f "rviz"
            sleep 2
            # Verify termination
            if [ $(ps aux | grep -E "ros2|gazebo|rviz" | grep -v grep | wc -l) -gt 0 ]; then
                echo -e "${RED}Warning: Some processes could not be terminated. Please check manually.${NC}"
                ps aux | grep -E "ros2|gazebo|rviz" | grep -v grep
                echo -e "${RED}Would you like to continue anyway? [y/N]${NC}"
                read -r force_continue
                if [[ ! "$force_continue" =~ ^([yY][eE][sS]|[yY])$ ]]; then
                    echo -e "${RED}Exiting.${NC}"
                    exit 1
                fi
            else
                echo -e "${GREEN}All processes terminated successfully.${NC}"
            fi
        else
            echo -e "${RED}Cannot continue with existing ROS2 processes. Please terminate them manually and try again.${NC}"
            exit 1
        fi
    else
        echo -e "${GREEN}No existing ROS2 processes found. Proceeding...${NC}"
    fi
}

# Function to source ROS2 environment
source_environment() {
    echo -e "${BLUE}Sourcing ROS2 and workspace environment...${NC}"
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    # Source LBR stack
    if [ -f /opt/grok/c6030l/workspace/lbr-stack/install/setup.bash ]; then
        source /opt/grok/c6030l/workspace/lbr-stack/install/setup.bash
        echo -e "${GREEN}LBR stack sourced successfully.${NC}"
    else
        echo -e "${RED}Error: LBR stack setup file not found. Please ensure it is installed correctly.${NC}"
        exit 1
    fi
    
    # Source this workspace
    if [ -f /opt/grok/c6030l/workspace/adaptive-simulation/install/setup.bash ]; then
        source /opt/grok/c6030l/workspace/adaptive-simulation/install/setup.bash
        echo -e "${GREEN}Adaptive simulation workspace sourced successfully.${NC}"
    else
        echo -e "${YELLOW}Warning: Adaptive simulation workspace setup file not found.${NC}"
        echo -e "${YELLOW}Have you built the workspace with 'colcon build'?${NC}"
        echo -e "${YELLOW}Would you like to build it now? [Y/n]${NC}"
        read -r build_response
        if [[ "$build_response" =~ ^([yY][eE][sS]|[yY]|"")$ ]]; then
            echo -e "${BLUE}Building workspace...${NC}"
            cd /opt/grok/c6030l/workspace/adaptive-simulation
            colcon build
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}Build successful. Sourcing workspace...${NC}"
                source /opt/grok/c6030l/workspace/adaptive-simulation/install/setup.bash
            else
                echo -e "${RED}Build failed. Please fix the issues and try again.${NC}"
                exit 1
            fi
        else
            echo -e "${RED}Cannot continue without sourcing the workspace. Exiting.${NC}"
            exit 1
        fi
    fi
}

# Function to present simulation options
select_simulation_mode() {
    echo -e "${BLUE}==============================================${NC}"
    echo -e "${BLUE}   Select Simulation Mode:                   ${NC}"
    echo -e "${BLUE}==============================================${NC}"
    echo -e "${YELLOW}1. Gazebo Simulation${NC} (Full physics simulation with Gazebo)"
    echo -e "${YELLOW}2. Mock Simulation${NC} (Lightweight simulation without Gazebo)"
    echo -e "${YELLOW}3. Custom Configuration${NC} (Specify custom launch arguments)"
    echo -e "${YELLOW}4. Exit${NC}"
    echo -e "${BLUE}==============================================${NC}"
    echo -e "Enter your choice [1-4]: "
    read -r choice
    
    case $choice in
        1)
            simulation_mode="gazebo"
            echo -e "${GREEN}Selected: Gazebo Simulation${NC}"
            ;;
        2)
            simulation_mode="mock"
            echo -e "${GREEN}Selected: Mock Simulation${NC}"
            ;;
        3)
            simulation_mode="custom"
            echo -e "${GREEN}Selected: Custom Configuration${NC}"
            ;;
        4)
            echo -e "${BLUE}Exiting.${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice. Exiting.${NC}"
            exit 1
            ;;
    esac
}

# Function to select obstacle configuration
select_obstacle_config() {
    echo -e "${BLUE}==============================================${NC}"
    echo -e "${BLUE}   Select Obstacle Configuration:            ${NC}"
    echo -e "${BLUE}==============================================${NC}"
    echo -e "${YELLOW}1. Default Obstacles${NC} (Standard test obstacles)"
    echo -e "${YELLOW}2. Empty Environment${NC} (No obstacles)"
    echo -e "${YELLOW}3. Custom Configuration File${NC} (Specify your own YAML file)"
    echo -e "${BLUE}==============================================${NC}"
    echo -e "Enter your choice [1-3]: "
    read -r obstacle_choice
    
    case $obstacle_choice in
        1)
            obstacle_config="$(rospack find adaptive_simulation)/config/obstacles.yaml"
            echo -e "${GREEN}Selected: Default Obstacles${NC}"
            ;;
        2)
            obstacle_config="$(rospack find adaptive_simulation)/config/empty_obstacles.yaml"
            echo -e "${GREEN}Selected: Empty Environment${NC}"
            ;;
        3)
            echo -e "Enter the full path to your obstacle configuration file: "
            read -r custom_obstacle_file
            if [ -f "$custom_obstacle_file" ]; then
                obstacle_config="$custom_obstacle_file"
                echo -e "${GREEN}Selected: Custom Configuration File: $obstacle_config${NC}"
            else
                echo -e "${RED}File not found. Using default obstacles.${NC}"
                obstacle_config="$(rospack find adaptive_simulation)/config/obstacles.yaml"
            fi
            ;;
        *)
            echo -e "${YELLOW}Invalid choice. Using default obstacles.${NC}"
            obstacle_config="$(rospack find adaptive_simulation)/config/obstacles.yaml"
            ;;
    esac
}

# Function to start the simulation
start_simulation() {
    echo -e "${BLUE}Starting Adaptive Surface Contact Simulation...${NC}"
    
    case $simulation_mode in
        gazebo)
            echo -e "${GREEN}Launching Gazebo Simulation...${NC}"
            ros2 launch adaptive_simulation simulation.launch.py \
                use_sim_time:=true \
                obstacle_config:="$obstacle_config"
            ;;
        mock)
            echo -e "${GREEN}Launching Mock Simulation...${NC}"
            # First start the mock interface
            echo -e "${YELLOW}Starting mock interface...${NC}"
            ros2 launch lbr_bringup mock.launch.py \
                ctrl:=joint_trajectory_controller \
                model:=iiwa14 \
                namespace:=/lbr &
            MOCK_PID=$!
            
            # Give it a moment to start
            sleep 3
            
            # Then start our simulation
            echo -e "${GREEN}Starting simulation components...${NC}"
            ros2 launch adaptive_simulation simulation.launch.py \
                use_sim_time:=true \
                obstacle_config:="$obstacle_config" \
                skip_gazebo:=true
            
            # Cleanup mock process when we're done
            kill $MOCK_PID
            ;;
        custom)
            echo -e "${YELLOW}Enter custom launch arguments (or press Enter for defaults):${NC}"
            read -r custom_args
            
            echo -e "${GREEN}Launching Custom Simulation...${NC}"
            ros2 launch adaptive_simulation simulation.launch.py \
                use_sim_time:=true \
                obstacle_config:="$obstacle_config" \
                $custom_args
            ;;
        *)
            echo -e "${RED}Invalid simulation mode. Exiting.${NC}"
            exit 1
            ;;
    esac
}

# Main execution
check_ros2_processes
source_environment
select_simulation_mode
select_obstacle_config
start_simulation

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}   Simulation Complete                       ${NC}"
echo -e "${BLUE}==============================================${NC}"
