# Path Smoothing and Trajectory Control for Differential Drive Robot

## Overview

This project implements a comprehensive path smoothing algorithm and trajectory tracking controller for a differential drive robot navigating through 2D waypoints. The system generates smooth trajectories from discrete waypoints and ensures accurate trajectory following using ROS2 and simulation.

## Features

- **Path Smoothing**: Converts discrete waypoints into smooth, continuous trajectories
- **Trajectory Generation**: Creates time-parameterized trajectories with velocity profiles
- **Trajectory Tracking Controller**: Implements a robust controller for accurate path following
- **ROS2 Integration**: Full ROS2 implementation with proper node architecture
- **Simulation Environment**: Complete simulation setup with visualization

## System Architecture

```
├── src/
|   ├── path_following/
│   |   ├── path_following/
│   |   │   ├── path_smoother_node.py          # Path smoothing algorithms
│   |   │   ├── trajectory_generator_node.py   # Trajectory generation with time parameterization
│   |   │   └── follower_node.py  # Trajectory tracking controller
|   ├── launch/
│       ├── run_navigation.launch.py          # Main launch file
|   ├── config/
│       └── nav_config.rviz            # rviz parameters
```

## Prerequisites

- **ROS2 Humble** (or later)
- **Ubuntu 22.04** (recommended)
- **C++17** or later
- **Python 3.8+**

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-nav2*
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Additional Dependencies
```bash
sudo apt install python3-pip
pip3 install matplotlib numpy scipy
```

## Installation & Setup

1. **Clone the repository:**
```bash
git clone <https://github.com/Ashwin001-fr/10x_path_following_assignment/tree/main>
cd 10x_task_ws
```

2. **Build the workspace:**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build the project
colcon build 

# Source the workspace
source install/setup.bash
```

3. **Set environment variables:**
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/models
```

## Usage

### Basic Simulation

1. **Launch the simulation:**
```bash
ros2 launch path_following run_navigation.launch.py
```


## Algorithm Design Choices

### 1. Path Smoothing Algorithm

**Chosen Approach: Catmull-Rom Spline Interpolation**

- **Rationale**: Provides C¹ continuity with guaranteed passage through all waypoints
- **Advantages**: 
  - Interpolating spline (passes exactly through control points)
  - Local control (each segment depends only on 4 nearby points)
  - Natural tangent calculation using neighboring points
  - No need for additional control point specification
- **Implementation**: Catmull-Rom spline with configurable tension parameter and uniform parameterization

**Alternative Considered**: Bezier curves - rejected due to global control issues


### 2. Trajectory Generation

**Chosen Approach: Trapezoidal Velocity Profile**

- **Rationale**: Provides smooth acceleration/deceleration with constant cruise velocity
- **Parameters**:
  - Maximum velocity: 0.8 m/s
  - Maximum acceleration: 0.5 m/s²
  - Sampling frequency: 50 Hz

### 3. Trajectory Tracking Controller

**Chosen Approach: Pure Pursuit Controller with Look-ahead Distance**

- **Rationale**: Simple, robust, and well-suited for differential drive robots
- **Parameters**:
  - Look-ahead distance: 0.5m (adaptive based on velocity)
  - Maximum angular velocity: 1.0 rad/s
  - Position tolerance: 0.1m


## Code Architecture Decisions

### 1. Modular Design
- **Separation of Concerns**: Each component (smoothing, generation, control) is independent
- **Interface-based Design**: Abstract base classes for easy algorithm swapping
- **Configuration-driven**: Parameters externalized to YAML files

### 2. ROS2 Integration
- **Node Composition**: Used ROS2 component nodes for better performance
- **QoS Profiles**: Configured appropriate QoS for real-time control
- **Lifecycle Management**: Implemented managed nodes for better state control
  

## Extension to Real Robot

### Hardware Considerations
1. **Sensor Integration**: 
   - IMU for orientation feedback
   - Wheel encoders for odometry
   - LiDAR for obstacle detection

2. **Communication**:
   - CAN bus for motor control
   - Ethernet for high-bandwidth sensor data

3. **Safety Systems**:
   - Emergency stop capabilities
   - Collision detection and avoidance
   - Watchdog timers

### Software Modifications
1. **Hardware Abstraction Layer**: Replace simulation interface with hardware drivers
2. **Sensor Fusion**: Implement Extended Kalman Filter for state estimation
3. **Robust Control**: Add adaptive control for varying terrain and disturbances
   

## Obstacle Avoidance Extension 

### Dynamic Window Approach (DWA)
- **Real-time Planning**: Generates collision-free velocities in real-time
- **Integration**: Seamlessly integrated with trajectory controller
- **Performance**: Maintains smooth motion while avoiding obstacles

### Implementation Features
- **Sensor Integration**: LiDAR-based obstacle detection
- **Cost Function**: Balances goal-reaching, obstacle avoidance, and smoothness
- **Recovery Behaviors**: Implements backup strategies when trapped

## AI Tools Utilized

### Development Workflow
1. **ChatGPT/Claude**: Algorithm research and code development and debugging support
   

## Troubleshooting

### Common Issues

1. **Simulation not starting**:
```bash
# Check Gazebo installation
gazebo --version
# Reinstall if necessary
sudo apt install gazebo
```

2. **Robot not moving**:
```bash
# Check topic connections
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel
```


## Demo Video

Link: https://drive.google.com/file/d/15BJW6kxmlNWQR6PDT7_5JGcYHXFMQdi7/view?usp=sharing
- Waypoint input and path smoothing visualization
- Robot following generated trajectory
- Performance metrics and plots


## Authors

[Ashwin Prem] - [ashwinprem2020@gmail.com]


## Acknowledgments
- Open-source robotics community for ROS2 packages
- AI tool developers for enhancing development workflow
