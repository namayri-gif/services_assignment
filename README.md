# Task 2: Obstacle Avoidance with ROS 2 Service

## Overview

This project implements an autonomous obstacle avoidance system for TurtleBot3 using ROS 2.

The robot uses LiDAR data from the `/scan` topic to detect nearby obstacles and make navigation decisions in real time. A custom ROS 2 service allows the robot's operating mode to be changed while the node is running.

The project is divided into two ROS 2 packages:

1. **turtlebot3_obstacle_avoidance** – Contains the obstacle avoidance logic and ROS 2 node.
2. **turtlebot3_obstacle_interfaces** – Contains the custom service definition used to control robot modes.

---

## Project Structure

```text
ros2_ws/src/

├── turtlebot3_obstacle_avoidance/
│   ├── turtlebot3_obstacle_avoidance/
│   │   ├── __init__.py
│   │   ├── avoidance_logic.py
│   │   └── obstacle_avoidance_node.py
│   │
│   ├── launch/
│   │   └── obstacle_avoidance.launch.py
│   │
│   ├── config/
│   │   └── params.yaml
│   │
│   ├── setup.py
│   ├── package.xml
│   └── README.md
│
└── turtlebot3_obstacle_interfaces/
    ├── srv/
    │   └── SetAvoidanceMode.srv
    │
    ├── CMakeLists.txt
    └── package.xml
```

---

## Features

* Autonomous obstacle avoidance using LiDAR
* State-based robot controller
* Forward navigation when path is clear
* Automatic turning toward the safest direction
* Reverse recovery when trapped
* Runtime mode switching through ROS 2 services
* Modular design separating robot logic from ROS interfaces

---

## Obstacle Avoidance Logic

The robot continuously processes LiDAR data from the `/scan` topic.

The scan is divided into three regions:

| Region | Purpose                         |
| ------ | ------------------------------- |
| Front  | Detect obstacles ahead          |
| Left   | Measure free space on the left  |
| Right  | Measure free space on the right |

The minimum valid distance from each region is used to determine the robot's next action.

---

## Robot States

### FORWARD

Default navigation state.

Behavior:

* Move forward
* Monitor front distance continuously

Transition:

```text
Front Distance < 0.5 m
```

Switches to TURN state.

---

### TURN

Obstacle avoidance state.

Behavior:

* Stop forward movement
* Rotate left or right
* Select the side with more free space

Transition:

```text
Front Distance > 1.0 m
```

Returns to FORWARD state.

---

### REVERSE

Emergency recovery state.

Behavior:

* Move backward slowly
* Rotate toward the safer side
* Continue until sufficient space is available

Used when:

* Front is blocked
* Left is unsafe
* Right is unsafe

---

## Safety Thresholds

| Parameter          | Value |
| ------------------ | ----- |
| Obstacle Detection | 0.5 m |
| Path Clear         | 1.0 m |
| Turning Safety     | 0.4 m |

---

## Robot Modes

The obstacle avoidance behavior supports three operating modes.

### AGGRESSIVE

Fast movement:

```text
Linear Speed = 0.5 m/s
```

---

### CAUTIOUS

Slow movement:

```text
Linear Speed = 0.2 m/s
```

---

### STOP

Emergency stop mode:

```text
linear.x = 0.0
angular.z = 0.0
```

The robot stops immediately when this mode is activated.

---

## Custom Service

A custom ROS 2 service is implemented in the separate interface package.

### Service File

```text
srv/SetAvoidanceMode.srv
```

### Service Definition

```text
string mode
---
bool success
string message
```

### Supported Modes

```text
AGGRESSIVE
CAUTIOUS
STOP
```

---

## Package Dependencies

The custom service was implemented in a separate package named:

```text
turtlebot3_obstacle_interfaces
```

This package was created using `ament_cmake`.

The obstacle avoidance package was configured to use this service by updating:

* package.xml
* setup.py

and adding the required dependencies so the service can be imported and used by the ROS 2 node.

---

## Build

Build both packages from the workspace root:

```bash
cd ~/ros2_ws

colcon build --packages-select \
turtlebot3_obstacle_interfaces \
turtlebot3_obstacle_avoidance

source install/setup.bash
```

---

## Launch

Run the obstacle avoidance node:

```bash
ros2 launch turtlebot3_obstacle_avoidance obstacle_avoidance.launch.py
```

---

## Service Examples

Switch to cautious mode:

```bash
ros2 service call /set_avoidance_mode \
turtlebot3_obstacle_interfaces/srv/SetAvoidanceMode \
"{mode: 'CAUTIOUS'}"
```

Switch to aggressive mode:

```bash
ros2 service call /set_avoidance_mode \
turtlebot3_obstacle_interfaces/srv/SetAvoidanceMode \
"{mode: 'AGGRESSIVE'}"
```

Emergency stop:

```bash
ros2 service call /set_avoidance_mode \
turtlebot3_obstacle_interfaces/srv/SetAvoidanceMode \
"{mode: 'STOP'}"
```

---

## Topics Used

| Topic    | Type                      | Purpose                 |
| -------- | ------------------------- | ----------------------- |
| /scan    | sensor_msgs/msg/LaserScan | LiDAR data              |
| /cmd_vel | geometry_msgs/msg/Twist   | Robot velocity commands |

---

## Verification

The system was tested using TurtleBot3 in simulation.

The following functionality was verified:

* Robot moves forward in open space
* Obstacles are detected using LiDAR
* Robot turns toward the safer direction
* Robot recovers when trapped
* Runtime mode changes work correctly
* Service communication functions properly
* Package builds and launches successfully

---

## Requirements

* ROS 2 Jazzy
* TurtleBot3 packages
* Gazebo Simulator
* sensor_msgs
* geometry_msgs
* rclpy

---

## Author

Rani Amayri

Task 2 – ROS 2 Obstacle Avoidance with Service-Based Mode Control
