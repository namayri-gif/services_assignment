# Task 3: TurtleBot3 ROS 2 Action Mission System

## Overview
This project implements a ROS 2 Services-based mission system for TurtleBot3 using Python.
The robot moves with the typical obstacle avoidance logic but has an override system to change its direction. The robot can move in all 4 directions {forward, reverse, left, right}
Rotate — using constant angular speed over a calculated duration
The project is divided into two ROS 2 packages:
`obstacle_direction_interfaces` — Contains the custom services definition
`obstacle_direction_controler` — Contains the obstacle avoidance logic

---
## Project Structure
```
services_assignment/
├── obstacle_direction_interfaces/
│   ├── srv/
│   │   └── SetDirection.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── obstacle_direction_controler/
│   ├── obstacle_direction_controler/
│   │   ├── __init__.py
│   │   ├── direction_autopilot_node.py
│   ├── resource/
│   │   └── obstacle_direction_controler
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
└── README.md
```
---
## Services Definition
File: `obstacle_direction_interfaces/srv/SetDirection.srv`
```
string direction
---
bool success
string message
```
## Field Meanings
Section	Field	Description
Request	`direction` change
Response	`success`	Immediate success message
Response	`message`	Immediate status message

---
## System Behavior

**Moving Forward**
- Publishes velocity commands to `/cmd_vel`
- Subscribes to `/scan` to read LiDAR distance data
- Moves at `forward_velocity = 0.2 m/s` when the front path is clear
- Transitions to `turn` state when `front_distance <= obstacle_threshold`

**Obstacle Detection**
- LiDAR scan is divided into three sectors: front, left, and right
- Each sector spans ±30° around its center angle
- Minimum valid range in each sector is used as the representative distance
- Invalid readings (< 0.1m or > max range) are filtered out

**Turning (Obstacle Avoidance)**
- Chooses turn direction based on which side has more clearance (`left >= right` → turn left)
- Rotates in place at `angular_velocity = 0.5 rad/s` until `front_distance > free_forward_threshold (1.0m)`
- Switches turn direction mid-rotation if the chosen side becomes unsafe (< 0.4m clearance)
- Returns to `forward` state once the front path is clear

**Trapped Recovery**
- If both left and right are below the safety threshold (0.4m), the robot has no safe turn direction
- Switches to `reverse` state and backs up at `-0.1 m/s` while rotating toward the roomier side
- Automatically recovers to `forward` state once the front clears and at least one side is safe

**Service-Commanded Directions (`/set_direction`)**
- Accepts `FORWARD`, `REVERSE`, `LEFT`, or `RIGHT` as string commands
- `FORWARD` / `REVERSE` — sets forward velocity to 0.2 m/s and re-enables obstacle avoidance thresholds
- `LEFT` — rotates counter-clockwise in place at `+0.5 rad/s` (positive z-axis)
- `RIGHT` — rotates clockwise in place at `-0.5 rad/s` (negative z-axis)
- Returns `success = true` and a confirmation message on valid input, `success = false` on unknown commands

**State Machine Overview**

| State | Trigger | Action |
|-------|---------|--------|
| `forward` | Default / path clear | Drive straight at 0.2 m/s |
| `turn` | Front obstacle detected | Rotate in place toward clearer side |
| `reverse` | Both sides blocked | Back up and rotate toward roomier side |
| `left` | Service command `LEFT` | Rotate counter-clockwise in place |
| `right` | Service command `RIGHT` | Rotate clockwise in place |
---
## Topics Used
Topic	Type	Purpose
`/cmd_vel`	`geometry_msgs/msg/Twist`	Robot velocity commands
`/scan`	`sensor_msgs/LaserScan`	Receive LiDAR data

---
## Build
Clone the repository into your workspace and build both packages:
```bash

cd services_assignment
colcon build
source install/setup.bash
```
---
## Notes
-Use the change direction call carefully as it can cause the robot to bumped into the walls if issued a wrong command

-Building a dedicated client node is unnecessary here. ROS 2 ships a built-in CLI tool that can call any service directly from the terminal, so testing the mode switch requires no extra Python code.

---
### Running the Code
Step 1 — Run obstacle_direction_controller
```bash
ros2 run obstacle_direction_controller direction_autopilot_node.py
```
Step 2 — Run the Services Package 
```bash
cd services_assignment
source install/setup.bash
ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'forward'}"
```
---
### Expected Output
Terminal 1:
```
<img width="1199" height="346" alt="image" src="https://github.com/user-attachments/assets/005b7b97-8305-41f2-91d9-8fbbea198443" />

```
Terminal 2:
```
<img width="1486" height="244" alt="image" src="https://github.com/user-attachments/assets/7472e7aa-b39a-49ac-bf69-0e8b3415587e" />

```
---
## Verification
The system was tested using TurtleBot3 in Gazebo simulation. The following was verified:
Robot moves forward the specified distance using odometry feedback
Robot rotates the specified angle using time-based control
Continuous feedback is published throughout execution
Timeout correctly aborts the mission and stops the robot
Final result message is received by the client

---
