# ROS 2 Generic Subscriber Package (Jazzy)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-green.svg)](https://docs.ros.org/en/jazzy/)  
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)  
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

This package provides a versatile ROS 2 generic subscriber in C++ for ROS 2 Jazzy, capable of subscribing to any topic and detecting its message type, accompanied by test publishers for validation.

## Overview

The `generic_subscriber_pkg` is designed to demonstrate a flexible subscription mechanism in ROS 2. It includes:
- A generic subscriber that dynamically detects and subscribes to any topic
- Test publishers for `std_msgs/String`, `std_msgs/Int32`, and `std_msgs/Float32` to verify functionality

This package is ideal for debugging, monitoring, or integrating with diverse ROS 2 systems.

## Features

- **Dynamic Subscription**: Adapts to any topic specified via parameter
- **Type Detection**: Identifies and logs the message type automatically
- **Test Publishers**: Publishes string, integer, and float messages at 1 Hz
- **Configurable**: Topic name adjustable via ROS parameters
- **Robust Design**: Retries topic detection with a 1-second interval until available
- **Lightweight**: Minimal dependencies for easy integration

## Prerequisites

- **ROS 2 Jazzy**: Ensure your system has ROS 2 Jazzy installed and sourced (`source /opt/ros/jazzy/setup.bash`)
- **C++17 Compiler**: e.g., g++ (typically included with ROS 2)
- **Colcon**: Build tool for ROS 2 packages
- **Git**: Optional, for cloning and version control

## Installation

1. **Create a Workspace**:
   ```bash
   mkdir -p ~/ros2_jazzy_ws/src
   cd ~/ros2_jazzy_ws
   ```

2. **Clone the Repository**:
   ```bash
   cd src
   git clone https://github.com/yassine-cherni/generic_subscriber_pkg.git
   ```

3. **Install Dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Package**:
   ```bash
   colcon build --packages-select generic_subscriber_pkg
   ```

5. **Source the Workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Example

Run each command in a separate terminal after sourcing the workspace (`source ~/ros2_jazzy_ws/install/setup.bash`).

### Start the Generic Subscriber
```bash
ros2 run generic_subscriber_pkg generic_subscriber --ros-args -p topic_name:=/test_topic
```
- The `-p topic_name:=/your_topic` flag lets you specify any topic (default: `/test_topic`).

### Test with Publishers
Test the subscriber with different message types by running one publisher at a time:

- **String Publisher**:
  ```bash
  ros2 run generic_subscriber_pkg string_publisher
  ```
  - Publishes `std_msgs/String` messages like "Hello, ROS 2! 0", "Hello, ROS 2! 1", etc.

- **Integer Publisher**:
  ```bash
  ros2 run generic_subscriber_pkg int_publisher
  ```
  - Publishes `std_msgs/Int32` messages incrementing from 0.

- **Float Publisher**:
  ```bash
  ros2 run generic_subscriber_pkg float_publisher
  ```
  - Publishes `std_msgs/Float32` messages incrementing by 0.1 (e.g., 0.0, 0.1, 0.2).

### Example Usage with Custom Topic
```bash
# Start subscriber on a custom topic
ros2 run generic_subscriber_pkg generic_subscriber --ros-args -p topic_name:=/chatter

# Publish to the custom topic
ros2 topic pub /chatter std_msgs/msg/String "data: 'Custom message'"
```

## Package Structure

```
generic_subscriber_pkg/
├── src/
│   ├── generic_subscriber.cpp  # Core subscriber logic
│   ├── string_publisher.cpp    # String message publisher
│   ├── int_publisher.cpp       # Integer message publisher
│   └── float_publisher.cpp     # Float message publisher
├── package.xml                 # Package manifest
├── CMakeLists.txt              # Build configuration
└── README.md                   # Documentation
```